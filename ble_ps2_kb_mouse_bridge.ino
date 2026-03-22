/*
 * BLE -> PS/2 Keyboard + Mouse Bridge  v2.1
 * ==========================================
 * ESP32-WROOM-32 | NimBLE-Arduino 2.x | Arduino IDE 2.x
 *
 * Connects simultaneously to a BLE HID keyboard AND a BLE HID mouse.
 * Forwards keyboard as PS/2 scan-code set 2.
 * Forwards mouse as PS/2 Standard / IntelliMouse / IntelliMouse Explorer
 * — protocol level negotiated automatically with the host, capped by 'proto' setting.
 *
 * Mouse BLE input supports:
 *   Standard 3/4/5-byte HID reports, Logitech 12-bit packed (MX Master, G502, M650)
 *   Buttons: Left, Right, Middle, Back, Forward
 *   Scroll wheel (vertical)
 *
 * PS/2 mouse output protocols (selectable via 'proto' command):
 *   0x00  Standard        3-byte packets, 3 buttons, no scroll
 *   0x03  IntelliMouse    4-byte packets, 3 buttons, scroll wheel
 *   0x04  Explorer        4-byte packets, 5 buttons (incl. Back/Fwd), scroll wheel [default]
 *
 * Wiring (each PS/2 port needs a bidirectional 3.3V <-> 5V level shifter):
 *
 *   Keyboard PS/2 socket (AT/5-pin DIN or 6-pin mini-DIN):
 *     CLK  <-> level shifter <-> GPIO19
 *     DATA <-> level shifter <-> GPIO18
 *
 *   Mouse PS/2 socket (6-pin mini-DIN):
 *     CLK  <-> level shifter <-> GPIO16
 *     DATA <-> level shifter <-> GPIO17
 *
 *   GND ----------- GND
 *   PS/2 +5V ------- VIN
 *
 * Serial commands (115200 baud):
 *   scan                  Scan BLE HID devices 10 s (shows KB / Mouse type)
 *   connect kb <mac>      Connect BLE keyboard and save to NVS
 *   connect mouse <mac>   Connect BLE mouse and save to NVS
 *   forget kb             Forget saved keyboard + bonds
 *   forget mouse          Forget saved mouse, reset proto to 0x04
 *   forget all            Forget both devices, reset all settings
 *   scale <1-64>          Mouse movement divisor              (default 4)
 *   flipy                 Toggle mouse Y-axis inversion       (default off)
 *   flipw                 Toggle scroll-wheel inversion       (default off)
 *   reportid <0-255>      BLE Report ID filter for mouse      (default 0 = auto)
 *   proto <0|3|4>         PS/2 protocol cap (0=Std 3=Intelli 4=Explorer) (default 4)
 *   status                Show connection status + all settings
 *   help                  Show command list
 *
 * Hotkeys (via connected BLE keyboard):
 *   LCtrl+LAlt+PrtSc            Type KB battery% + Mouse battery% via PS/2
 *   LCtrl+LAlt+LShift+PrtSc     Type full status block via PS/2
 *
 * Timing / reliability notes:
 *   - PS/2 keyboard and mouse each run on independent FreeRTOS tasks (Core 0).
 *   - BLE battery keepalives (readValue) run in bleDaemonTask to avoid
 *     blocking loop() and delaying PS/2 Reset responses.
 *   - Packet queue is flushed on PS/2 Reset (0xFF) before sending BAT,
 *     preventing stale data from corrupting host driver state after reload.
 *   - Serial input uses a non-blocking character accumulator — loop() is never
 *     held waiting for input, so PS/2 timing is unaffected.
 */

// Two simultaneous BLE clients need more NimBLE host stack than the default 4 kB.
#ifndef CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE
  #define CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE 8192
#endif

#include <NimBLEDevice.h>
#include <Preferences.h>
#include <set>
#include "esp_wifi.h"

// LED pending update — set by PS/2 keyboard host task, consumed by loop()
static volatile uint8_t pendingLedMask = 0xFF; // 0xFF = no pending update

// ════════════════════════════════════════════════════════════════════════════════
//  LOCK-FREE LOG QUEUE  (Core 0 → Core 1)
// ════════════════════════════════════════════════════════════════════════════════
// HardwareSerial is not safe to call from multiple cores simultaneously.
// All Serial output from Core 0 tasks must go through this queue.
// loop() on Core 1 drains it every iteration via logDrain().
//
// Design constraints:
//   • logQ() is called from ps2 host tasks while they hold the PS/2 mutex.
//     vsnprintf() inside a mutex would delay the mutex and starve ps2send task.
//   • Solution: store the raw message as a fixed string — logQ() only does
//     a memcpy of an already-formatted string, which is ~10 ns.
//   • Callers pre-format with snprintf into a local buffer, then call logQ().
//   • For simple string literals, logQ_str() copies directly.
//
// Single-producer / single-consumer ring buffer — no mutex needed.
// If the queue is full the message is silently dropped (non-blocking).

#define LOG_QUEUE_SIZE  32
#define LOG_MSG_LEN     80

static char     _logBuf[LOG_QUEUE_SIZE][LOG_MSG_LEN];
static volatile uint32_t _logHead = 0;  // written by Core 0
static volatile uint32_t _logTail = 0;  // read    by Core 1

// Enqueue a pre-formatted C string. IRAM — safe from any task/ISR.
static void IRAM_ATTR logQ_str(const char* s) {
  uint32_t next = (_logHead + 1) % LOG_QUEUE_SIZE;
  if (next == _logTail) return;           // full — drop
  strncpy(_logBuf[_logHead], s, LOG_MSG_LEN - 1);
  _logBuf[_logHead][LOG_MSG_LEN - 1] = '\0';
  _logHead = next;                        // publish atomically
}

// Convenience wrapper — formats into a local stack buffer, then enqueues.
// Do NOT call while holding a FreeRTOS mutex (snprintf can be slow).
// For calls inside reply_to_host() use logQ_str() with a pre-built string.
static void logQ(const char* fmt, ...) {
  char tmp[LOG_MSG_LEN];
  va_list ap; va_start(ap, fmt); vsnprintf(tmp, LOG_MSG_LEN, fmt, ap); va_end(ap);
  logQ_str(tmp);
}

// Drain from loop() (Core 1) only.
// Non-blocking: prints only as many messages as fit in the UART TX buffer
// without stalling. Remaining messages stay queued for the next loop() call.
static void logDrain() {
  while (_logTail != _logHead) {
    const char* s = _logBuf[_logTail];
    int len = strlen(s);
    if (Serial.availableForWrite() < len) break; // TX buffer would block — defer
    Serial.print(s);
    _logTail = (_logTail + 1) % LOG_QUEUE_SIZE;
  }
}

// ════════════════════════════════════════════════════════════════════════════════
//  PS/2 TIMING + COMMON LOW-LEVEL HELPERS
// ════════════════════════════════════════════════════════════════════════════════

#define PS2_CLK_HALF_US       40    // half clock period µs  (spec 30–50)
#define PS2_CLK_QUARTER_US    20
#define PS2_BYTE_INTERVAL_US 500    // gap between bytes
#define PS2_HOST_POLL_MS       2    // how often to check for host commands
#define PS2_PACKET_QUEUE_LEN  32

static uint64_t ps2_micros() { return esp_timer_get_time(); }
static void ps2_delay_us(uint32_t us) {
  uint64_t end = ps2_micros() + us;
  while (ps2_micros() < end) { asm volatile("nop"); }
}

// Generic PS/2 packet — reused by both keyboard and mouse
struct PS2Packet {
  uint8_t len;
  uint8_t data[16];  // 16 bytes: PAUSE make code is 8 bytes (longest sequence)
};

// ════════════════════════════════════════════════════════════════════════════════
//  PS2KEYBOARD — declaration
//  (identical to ble_ps2_bridge v1.6 — no changes)
// ════════════════════════════════════════════════════════════════════════════════

typedef enum {
  K_A, K_B, K_C, K_D, K_E, K_F, K_G, K_H, K_I, K_J, K_K, K_L, K_M,
  K_N, K_O, K_P, K_Q, K_R, K_S, K_T, K_U, K_V, K_W, K_X, K_Y, K_Z,
  K_0, K_1, K_2, K_3, K_4, K_5, K_6, K_7, K_8, K_9,
  K_BACKQUOTE, K_MINUS, K_EQUALS, K_BACKSLASH, K_BACKSPACE, K_SPACE,
  K_TAB, K_CAPSLOCK,
  K_LSHIFT, K_LCTRL, K_LSUPER, K_LALT,
  K_RSHIFT, K_RCTRL, K_RSUPER, K_RALT, K_MENU,
  K_RETURN, K_ESCAPE,
  K_F1, K_F2, K_F3, K_F4, K_F5, K_F6, K_F7, K_F8, K_F9, K_F10, K_F11, K_F12,
  K_PRINT, K_SCROLLOCK, K_PAUSE,
  K_LEFTBRACKET, K_INSERT, K_HOME, K_PAGEUP,
  K_DELETE, K_END, K_PAGEDOWN,
  K_UP, K_LEFT, K_DOWN, K_RIGHT,
  K_NUMLOCK,
  K_KP_DIVIDE, K_KP_MULTIPLY, K_KP_MINUS, K_KP_PLUS, K_KP_ENTER, K_KP_PERIOD,
  K_KP0, K_KP1, K_KP2, K_KP3, K_KP4, K_KP5, K_KP6, K_KP7, K_KP8, K_KP9,
  K_RIGHTBRACKET, K_SEMICOLON, K_QUOTE, K_COMMA, K_PERIOD, K_SLASH,
  K_ACPI_POWER, K_ACPI_SLEEP, K_ACPI_WAKE,
  K_MEDIA_NEXT_TRACK, K_MEDIA_PREV_TRACK, K_MEDIA_STOP, K_MEDIA_PLAY_PAUSE,
  K_MEDIA_MUTE, K_MEDIA_VOLUME_UP, K_MEDIA_VOLUME_DOWN,
  K_MEDIA_MEDIA_SELECT, K_MEDIA_EMAIL, K_MEDIA_CALC, K_MEDIA_MY_COMPUTER,
  K_MEDIA_WWW_SEARCH, K_MEDIA_WWW_HOME, K_MEDIA_WWW_BACK, K_MEDIA_WWW_FORWARD,
  K_MEDIA_WWW_STOP, K_MEDIA_WWW_REFRESH, K_MEDIA_WWW_FAVORITES,
  PS2KB_KEY_COUNT
} PS2Key;

class PS2Keyboard {
public:
  PS2Keyboard(int clk, int data);
  void begin();
  void keyHid_send(uint8_t hidkey, bool keyDown);
  enum class BusState { IDLE, INHIBITED, HOST_REQUEST_TO_SEND };
  BusState get_bus_state();
  int  ps2_write(uint8_t data);
  int  ps2_write_wait_idle(uint8_t data, uint64_t timeout_us = 100000);
  int  ps2_read(uint8_t *data, uint64_t timeout_ms = 0);
  void reply_to_host(uint8_t cmd);
  int  send_packet(PS2Packet *pkt);
  // Flush pending packets — call from reply_to_host(0xFF) before sending BAT,
  // while mutex is held, so send task cannot dequeue stale data after reset.
  void flushQueue() { if (_queue) xQueueReset(_queue); }
  SemaphoreHandle_t get_mutex()  { return _mutex; }
  QueueHandle_t     get_queue()  { return _queue; }
private:
  int _clk, _dat;
  SemaphoreHandle_t _mutex;
  QueueHandle_t     _queue;
  TaskHandle_t      _task_host;
  TaskHandle_t      _task_send;
  bool              _data_reporting = true;
  void _gohi(int pin);
  void _golo(int pin);
  void _ack();
  void _keydown(PS2Key key);
  void _keyup(PS2Key key);
};

void ps2kb_task_host(void *arg);
void ps2kb_task_send(void *arg);

// ════════════════════════════════════════════════════════════════════════════════
//  PS2MOUSE — declaration
//  PS/2 mouse device: stream mode, IntelliMouse scroll-wheel detection
// ════════════════════════════════════════════════════════════════════════════════

class PS2Mouse {
public:
  PS2Mouse(int clk, int dat) : _clk(clk), _dat(dat) {}
  void begin();

  // Called from processMouseMovement() — queues a PS/2 mouse packet.
  // dy_hid: BLE/HID Y (positive = down); internally negated for PS/2 (positive = up).
  void sendMovement(int8_t dx, int8_t dy_hid, int8_t dw, uint8_t btns);

  // Internal — used by FreeRTOS tasks
  enum class BusState { IDLE, INHIBITED, HOST_REQUEST_TO_SEND };
  BusState get_bus_state();
  int  ps2_write(uint8_t data);
  int  ps2_write_wait_idle(uint8_t data, uint64_t timeout_us = 100000);
  int  ps2_read(uint8_t *value, uint64_t timeout_ms = 0);
  void reply_to_host(uint8_t cmd);
  int  send_packet(PS2Packet *pkt);
  // Flush pending packets — call from reply_to_host(0xFF) before sending BAT.
  void flushQueue() { if (_queue) xQueueReset(_queue); }
  SemaphoreHandle_t get_mutex() { return _mutex; }
  QueueHandle_t     get_queue() { return _queue; }

private:
  int _clk, _dat;
  SemaphoreHandle_t _mutex  = nullptr;
  QueueHandle_t     _queue  = nullptr;
  TaskHandle_t      _task_host = nullptr;
  TaskHandle_t      _task_send = nullptr;
  bool    _dataReporting = false;
  bool    _intelliMouse  = false;  // ID=0x03: 4-byte packets + scroll wheel
  bool    _explorerMouse = false;  // ID=0x04: 4-byte packets + scroll + B4/B5 (Back/Forward)
  uint8_t _sampleHistory[3] = {0, 0, 0}; // last 3 Set Sample Rate values

  void _gohi(int pin);
  void _golo(int pin);
  void _ack();
};

void ps2mouse_task_host(void *arg);
void ps2mouse_task_send(void *arg);

// ════════════════════════════════════════════════════════════════════════════════
//  PS2KEYBOARD — implementation  (identical to v1.6)
// ════════════════════════════════════════════════════════════════════════════════

static const uint8_t MAKE[][9] = {
  {1,0x1C},{1,0x32},{1,0x21},{1,0x23},{1,0x24},{1,0x2B},{1,0x34},{1,0x33},
  {1,0x43},{1,0x3B},{1,0x42},{1,0x4B},{1,0x3A},{1,0x31},{1,0x44},{1,0x4D},
  {1,0x15},{1,0x2D},{1,0x1B},{1,0x2C},{1,0x3C},{1,0x2A},{1,0x1D},{1,0x22},
  {1,0x35},{1,0x1A},
  {1,0x45},{1,0x16},{1,0x1E},{1,0x26},{1,0x25},{1,0x2E},{1,0x36},{1,0x3D},
  {1,0x3E},{1,0x46},
  {1,0x0E},{1,0x4E},{1,0x55},{1,0x5D},{1,0x66},{1,0x29},
  {1,0x0D},{1,0x58},
  {1,0x12},{1,0x14},{2,0xE0,0x1F},{1,0x11},
  {1,0x59},{2,0xE0,0x14},{2,0xE0,0x27},{2,0xE0,0x11},{2,0xE0,0x2F},
  {1,0x5A},{1,0x76},
  {1,0x05},{1,0x06},{1,0x04},{1,0x0C},{1,0x03},{1,0x0B},
  {1,0x83},{1,0x0A},{1,0x01},{1,0x09},{1,0x78},{1,0x07},
  {4,0xE0,0x12,0xE0,0x7C},{1,0x7E},{8,0xE1,0x14,0x77,0xE1,0xF0,0x14,0xF0,0x77},
  {1,0x54},
  {2,0xE0,0x70},{2,0xE0,0x6C},{2,0xE0,0x7D},
  {2,0xE0,0x71},{2,0xE0,0x69},{2,0xE0,0x7A},
  {2,0xE0,0x75},{2,0xE0,0x6B},{2,0xE0,0x72},{2,0xE0,0x74},
  {1,0x77},
  {2,0xE0,0x4A},{1,0x7C},{1,0x7B},{1,0x79},{2,0xE0,0x5A},{1,0x71},
  {1,0x70},{1,0x69},{1,0x72},{1,0x7A},{1,0x6B},{1,0x73},{1,0x74},
  {1,0x6C},{1,0x75},{1,0x7D},
  {1,0x5B},{1,0x4C},{1,0x52},{1,0x41},{1,0x49},{1,0x4A},
  {2,0xE0,0x37},{2,0xE0,0x3F},{2,0xE0,0x5E},
  {2,0xE0,0x4D},{2,0xE0,0x15},{2,0xE0,0x3B},{2,0xE0,0x34},
  {2,0xE0,0x23},{2,0xE0,0x32},{2,0xE0,0x21},
  {2,0xE0,0x50},{2,0xE0,0x48},{2,0xE0,0x2B},{2,0xE0,0x40},
  {2,0xE0,0x10},{2,0xE0,0x3A},{2,0xE0,0x38},{2,0xE0,0x30},
  {2,0xE0,0x28},{2,0xE0,0x20},{2,0xE0,0x18},
};

PS2Keyboard::PS2Keyboard(int clk, int data) : _clk(clk), _dat(data) {}

void PS2Keyboard::_gohi(int pin) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
  gpio_set_level((gpio_num_t)pin, 1);
}
void PS2Keyboard::_golo(int pin) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT_OD);
  gpio_set_level((gpio_num_t)pin, 0);
}

PS2Keyboard::BusState PS2Keyboard::get_bus_state() {
  if (gpio_get_level((gpio_num_t)_clk) == 0) return BusState::INHIBITED;
  if (gpio_get_level((gpio_num_t)_dat) == 0) return BusState::HOST_REQUEST_TO_SEND;
  return BusState::IDLE;
}

int PS2Keyboard::ps2_write(uint8_t data) {
  if (get_bus_state() != BusState::IDLE) return -1;
  uint8_t parity = 1;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  taskENTER_CRITICAL(&mux);
  _golo(_dat);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_clk);
  ps2_delay_us(PS2_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  for (int i = 0; i < 8; i++) {
    if (data & 0x01) _gohi(_dat); else _golo(_dat);
    ps2_delay_us(PS2_CLK_QUARTER_US);
    _golo(_clk);
    ps2_delay_us(PS2_CLK_HALF_US);
    _gohi(_clk);
    ps2_delay_us(PS2_CLK_QUARTER_US);
    parity ^= (data & 0x01);
    data >>= 1;
  }
  if (parity) _gohi(_dat); else _golo(_dat);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_clk);
  ps2_delay_us(PS2_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _gohi(_dat);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_clk);
  ps2_delay_us(PS2_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  taskEXIT_CRITICAL(&mux);
  return 0;
}

int PS2Keyboard::ps2_write_wait_idle(uint8_t data, uint64_t timeout_us) {
  // Yield to IDLE task every ~1 ms so the watchdog is fed.
  // Pure busy-spin at priority 15 on Core 0 starves IDLE0 → watchdog crash.
  // 1 ms yield is fine: clock-inhibit periods are in the ms range, not µs.
  uint64_t start = ps2_micros();
  while (get_bus_state() != BusState::IDLE) {
    if (ps2_micros() - start > timeout_us) return -1;
    vTaskDelay(1); // yield for 1 tick (~1 ms) — feeds watchdog, allows IDLE0
  }
  return ps2_write(data);
}

int PS2Keyboard::ps2_read(uint8_t *value, uint64_t timeout_ms) {
  unsigned long wait_start = millis();
  while (get_bus_state() != BusState::HOST_REQUEST_TO_SEND) {
    if (timeout_ms > 0 && (millis() - wait_start) > timeout_ms) return -1;
    vTaskDelay(1);
  }
  uint8_t data = 0, bit = 1;
  uint8_t calc_parity = 1, recv_parity = 0;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  taskENTER_CRITICAL(&mux);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_clk);
  ps2_delay_us(PS2_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  for (int i = 0; i < 8; i++) {
    if (gpio_get_level((gpio_num_t)_dat)) { data |= bit; calc_parity ^= 1; }
    bit <<= 1;
    ps2_delay_us(PS2_CLK_QUARTER_US);
    _golo(_clk);
    ps2_delay_us(PS2_CLK_HALF_US);
    _gohi(_clk);
    ps2_delay_us(PS2_CLK_QUARTER_US);
  }
  if (gpio_get_level((gpio_num_t)_dat)) recv_parity = 1;
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_clk);
  ps2_delay_us(PS2_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_dat);
  _golo(_clk);
  ps2_delay_us(PS2_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _gohi(_dat);
  taskEXIT_CRITICAL(&mux);
  *value = data;
  return (recv_parity == calc_parity) ? 0 : -2;
}

void PS2Keyboard::_ack() {
  ps2_delay_us(PS2_BYTE_INTERVAL_US);
  ps2_write(0xFA);
  ps2_delay_us(PS2_BYTE_INTERVAL_US);
}

void PS2Keyboard::reply_to_host(uint8_t cmd) {
  uint8_t val;
  // Per-command logging removed — floods logQ at hundreds of entries/s
  // when host actively polls. Only important events (LED, Reset) are logged below.
  switch (cmd) {
    case 0xFF:
      _data_reporting = false;
      // Flush any queued scan-code packets BEFORE sending BAT.
      // The send task is blocked on our mutex right now — xQueueReset is safe here.
      // Without this, stale packets would follow the BAT and corrupt the host's
      // state machine (exact PS/2 analogy of the RS-232 'M'-after-timeout bug).
      flushQueue();
      _ack();
      while (ps2_write(0xAA) != 0) vTaskDelay(1);
      _data_reporting = true;
      break;
    case 0xFE: _ack(); break;
    case 0xF6: _ack(); break;
    case 0xF5: _data_reporting = false; _ack(); break;
    case 0xF4: _data_reporting = true;  _ack(); break;
    case 0xF3:
      _ack();
      if (ps2_read(&val) == 0) _ack();
      break;
    case 0xF2:
      _ack();
      while (ps2_write(0xAB) != 0) vTaskDelay(1);
      while (ps2_write(0x83) != 0) vTaskDelay(1);
      break;
    case 0xF0:
      _ack();
      if (ps2_read(&val) == 0) _ack();
      break;
    case 0xEE:
      ps2_delay_us(PS2_BYTE_INTERVAL_US);
      ps2_write(0xEE);
      ps2_delay_us(PS2_BYTE_INTERVAL_US);
      break;
    case 0xED:
      ps2_delay_us(PS2_BYTE_INTERVAL_US);
      while (ps2_write(0xFA) != 0) vTaskDelay(1);
      ps2_delay_us(PS2_BYTE_INTERVAL_US);
      if (ps2_read(&val, 10) == 0) {
        ps2_delay_us(PS2_BYTE_INTERVAL_US);
        while (ps2_write(0xFA) != 0) vTaskDelay(1);
        ps2_delay_us(PS2_BYTE_INTERVAL_US);
        // PS/2 LED bits: 0=ScrollLock 1=NumLock 2=CapsLock
        // BLE HID LED bits: 0=NumLock  1=CapsLock  2=ScrollLock
        { char _t[40]; snprintf(_t,40,"[PS2KB] LED mask 0x%02X\n",val); logQ_str(_t); }
        uint8_t bleLed = 0;
        if (val & 0x02) bleLed |= 0x01;
        if (val & 0x04) bleLed |= 0x02;
        if (val & 0x01) bleLed |= 0x04;
        pendingLedMask = bleLed;
      }
      break;
    default: _ack(); break;
  }
}

void PS2Keyboard::begin() {
  gpio_config_t io = {};
  io.intr_type    = GPIO_INTR_DISABLE;
  io.mode         = GPIO_MODE_OUTPUT_OD;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.pull_up_en   = GPIO_PULLUP_DISABLE;
  io.pin_bit_mask = (1ULL << _clk); gpio_config(&io);
  io.pin_bit_mask = (1ULL << _dat); gpio_config(&io);
  _gohi(_clk); _gohi(_dat);
  _mutex = xSemaphoreCreateMutex();
  _queue = xQueueCreate(PS2_PACKET_QUEUE_LEN, sizeof(PS2Packet));
  xSemaphoreTake(_mutex, portMAX_DELAY);
  ps2_delay_us(PS2_BYTE_INTERVAL_US);
  vTaskDelay(pdMS_TO_TICKS(200));
  ps2_write(0xAA);
  xSemaphoreGive(_mutex);
  xTaskCreatePinnedToCore(ps2kb_task_host, "ps2kbhost", 4096, this,  8, &_task_host, 0);  // Core 0, pri 8
  // Send task on Core 0, NOT Core 1.
  // ps2_write() uses taskENTER_CRITICAL which masks interrupts on the running core.
  // UART RX interrupts (Serial) run on Core 1 — moving send task to Core 0 means
  // PS/2 byte transmission never masks UART RX, so Serial console always works.
  // Priority 15 preempts BLE host stack (~pri 5) when a scan code needs sending.
  xTaskCreatePinnedToCore(ps2kb_task_send, "ps2kbsend", 4096, this, 15, &_task_send, 0);
}

int PS2Keyboard::send_packet(PS2Packet *pkt) {
  return (xQueueSend(_queue, pkt, 0) == pdTRUE) ? 0 : -1;
}

void PS2Keyboard::_keydown(PS2Key key) {
  if (!_data_reporting) return;
  PS2Packet pkt;
  pkt.len = MAKE[key][0];
  for (int i = 0; i < pkt.len; i++) pkt.data[i] = MAKE[key][i + 1];
  send_packet(&pkt);
}

void PS2Keyboard::_keyup(PS2Key key) {
  if (!_data_reporting) return;
  if (key == K_PAUSE) return;
  PS2Packet pkt; pkt.len = 0;
  uint8_t makelen = MAKE[key][0];
  const uint8_t *make = &MAKE[key][1];
  int i = 0;
  while (i < makelen - 1 && (make[i] == 0xE0 || make[i] == 0xE1))
    pkt.data[pkt.len++] = make[i++];
  pkt.data[pkt.len++] = 0xF0;
  while (i < makelen) pkt.data[pkt.len++] = make[i++];
  send_packet(&pkt);
}

void PS2Keyboard::keyHid_send(uint8_t hid, bool down) {
  PS2Key key;
  switch (hid) {
    case 0x04:key=K_A;break;     case 0x05:key=K_B;break;
    case 0x06:key=K_C;break;     case 0x07:key=K_D;break;
    case 0x08:key=K_E;break;     case 0x09:key=K_F;break;
    case 0x0A:key=K_G;break;     case 0x0B:key=K_H;break;
    case 0x0C:key=K_I;break;     case 0x0D:key=K_J;break;
    case 0x0E:key=K_K;break;     case 0x0F:key=K_L;break;
    case 0x10:key=K_M;break;     case 0x11:key=K_N;break;
    case 0x12:key=K_O;break;     case 0x13:key=K_P;break;
    case 0x14:key=K_Q;break;     case 0x15:key=K_R;break;
    case 0x16:key=K_S;break;     case 0x17:key=K_T;break;
    case 0x18:key=K_U;break;     case 0x19:key=K_V;break;
    case 0x1A:key=K_W;break;     case 0x1B:key=K_X;break;
    case 0x1C:key=K_Y;break;     case 0x1D:key=K_Z;break;
    case 0x1E:key=K_1;break;     case 0x1F:key=K_2;break;
    case 0x20:key=K_3;break;     case 0x21:key=K_4;break;
    case 0x22:key=K_5;break;     case 0x23:key=K_6;break;
    case 0x24:key=K_7;break;     case 0x25:key=K_8;break;
    case 0x26:key=K_9;break;     case 0x27:key=K_0;break;
    case 0x28:key=K_RETURN;break;    case 0x29:key=K_ESCAPE;break;
    case 0x2A:key=K_BACKSPACE;break; case 0x2B:key=K_TAB;break;
    case 0x2C:key=K_SPACE;break;     case 0x2D:key=K_MINUS;break;
    case 0x2E:key=K_EQUALS;break;    case 0x2F:key=K_LEFTBRACKET;break;
    case 0x30:key=K_RIGHTBRACKET;break; case 0x31:key=K_BACKSLASH;break;
    case 0x33:key=K_SEMICOLON;break; case 0x34:key=K_QUOTE;break;
    case 0x35:key=K_BACKQUOTE;break; case 0x36:key=K_COMMA;break;
    case 0x37:key=K_PERIOD;break;    case 0x38:key=K_SLASH;break;
    case 0x39:key=K_CAPSLOCK;break;
    case 0x3A:key=K_F1;break;  case 0x3B:key=K_F2;break;
    case 0x3C:key=K_F3;break;  case 0x3D:key=K_F4;break;
    case 0x3E:key=K_F5;break;  case 0x3F:key=K_F6;break;
    case 0x40:key=K_F7;break;  case 0x41:key=K_F8;break;
    case 0x42:key=K_F9;break;  case 0x43:key=K_F10;break;
    case 0x44:key=K_F11;break; case 0x45:key=K_F12;break;
    case 0x46:key=K_PRINT;break;   case 0x47:key=K_SCROLLOCK;break;
    case 0x48:key=K_PAUSE;break;   case 0x49:key=K_INSERT;break;
    case 0x4A:key=K_HOME;break;    case 0x4B:key=K_PAGEUP;break;
    case 0x4C:key=K_DELETE;break;  case 0x4D:key=K_END;break;
    case 0x4E:key=K_PAGEDOWN;break;
    case 0x4F:key=K_RIGHT;break;   case 0x50:key=K_LEFT;break;
    case 0x51:key=K_DOWN;break;    case 0x52:key=K_UP;break;
    case 0x53:key=K_NUMLOCK;break;
    case 0x54:key=K_KP_DIVIDE;break;   case 0x55:key=K_KP_MULTIPLY;break;
    case 0x56:key=K_KP_MINUS;break;    case 0x57:key=K_KP_PLUS;break;
    case 0x58:key=K_KP_ENTER;break;
    case 0x59:key=K_KP1;break; case 0x5A:key=K_KP2;break;
    case 0x5B:key=K_KP3;break; case 0x5C:key=K_KP4;break;
    case 0x5D:key=K_KP5;break; case 0x5E:key=K_KP6;break;
    case 0x5F:key=K_KP7;break; case 0x60:key=K_KP8;break;
    case 0x61:key=K_KP9;break; case 0x62:key=K_KP0;break;
    case 0x63:key=K_KP_PERIOD;break; case 0x65:key=K_MENU;break;
    case 0x66:key=K_ACPI_POWER;break;
    case 0x74:key=K_MEDIA_PLAY_PAUSE;break;
    case 0x78:key=K_MEDIA_STOP;break;
    case 0x7F:key=K_MEDIA_MUTE;break;
    case 0x80:key=K_MEDIA_VOLUME_UP;break;
    case 0x81:key=K_MEDIA_VOLUME_DOWN;break;
    case 0xE0:key=K_LCTRL;break;  case 0xE1:key=K_LSHIFT;break;
    case 0xE2:key=K_LALT;break;   case 0xE3:key=K_LSUPER;break;
    case 0xE4:key=K_RCTRL;break;  case 0xE5:key=K_RSHIFT;break;
    case 0xE6:key=K_RALT;break;   case 0xE7:key=K_RSUPER;break;
    default: return;
  }
  if (down) _keydown(key); else _keyup(key);
}

void ps2kb_task_host(void *arg) {
  PS2Keyboard *kb = (PS2Keyboard *)arg;
  while (true) {
    xSemaphoreTake(kb->get_mutex(), portMAX_DELAY);
    if (kb->get_bus_state() == PS2Keyboard::BusState::HOST_REQUEST_TO_SEND) {
      uint8_t cmd;
      if (kb->ps2_read(&cmd) == 0) kb->reply_to_host(cmd);
    }
    xSemaphoreGive(kb->get_mutex());
    vTaskDelay(pdMS_TO_TICKS(PS2_HOST_POLL_MS));
  }
}

void ps2kb_task_send(void *arg) {
  PS2Keyboard *kb = (PS2Keyboard *)arg;
  while (true) {
    PS2Packet pkt;
    if (xQueueReceive(kb->get_queue(), &pkt, portMAX_DELAY) == pdTRUE) {
      xSemaphoreTake(kb->get_mutex(), portMAX_DELAY);
      ps2_delay_us(PS2_BYTE_INTERVAL_US);
      for (int i = 0; i < pkt.len; i++) {
        // Never silently drop a byte — retry until sent.
        // If a byte is dropped mid-sequence (e.g. E0 + scancode for arrow key),
        // the 8042 stalls waiting for the missing byte → stuck/wrong keys until reset.
        // 8042 may inhibit clock while CPU reads port 0x60 (IRQ1 handler latency).
        // 100 ms timeout in ps2_write_wait_idle covers any realistic handler.
        while (kb->ps2_write_wait_idle(pkt.data[i]) != 0) {
          ps2_delay_us(PS2_BYTE_INTERVAL_US);
        }
        if (i < pkt.len - 1)
          ps2_delay_us(PS2_BYTE_INTERVAL_US);
      }
      xSemaphoreGive(kb->get_mutex());
    }
  }
}

// ════════════════════════════════════════════════════════════════════════════════
//  PS2MOUSE — implementation
// ════════════════════════════════════════════════════════════════════════════════

// Forward declaration — g_ps2ProtoMode is defined in the main sketch section below.
// reply_to_host() reads it to cap the negotiated PS/2 protocol level.
extern volatile uint8_t g_ps2ProtoMode;

void PS2Mouse::_gohi(int pin) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
  gpio_set_level((gpio_num_t)pin, 1);
}
void PS2Mouse::_golo(int pin) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT_OD);
  gpio_set_level((gpio_num_t)pin, 0);
}

PS2Mouse::BusState PS2Mouse::get_bus_state() {
  if (gpio_get_level((gpio_num_t)_clk) == 0) return BusState::INHIBITED;
  if (gpio_get_level((gpio_num_t)_dat) == 0) return BusState::HOST_REQUEST_TO_SEND;
  return BusState::IDLE;
}

int PS2Mouse::ps2_write(uint8_t data) {
  if (get_bus_state() != BusState::IDLE) return -1;
  uint8_t parity = 1;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  taskENTER_CRITICAL(&mux);
  _golo(_dat);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_clk); ps2_delay_us(PS2_CLK_HALF_US); _gohi(_clk); ps2_delay_us(PS2_CLK_QUARTER_US);
  for (int i = 0; i < 8; i++) {
    if (data & 0x01) _gohi(_dat); else _golo(_dat);
    ps2_delay_us(PS2_CLK_QUARTER_US);
    _golo(_clk); ps2_delay_us(PS2_CLK_HALF_US); _gohi(_clk); ps2_delay_us(PS2_CLK_QUARTER_US);
    parity ^= (data & 0x01);
    data >>= 1;
  }
  if (parity) _gohi(_dat); else _golo(_dat);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_clk); ps2_delay_us(PS2_CLK_HALF_US); _gohi(_clk); ps2_delay_us(PS2_CLK_QUARTER_US);
  _gohi(_dat); ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_clk); ps2_delay_us(PS2_CLK_HALF_US); _gohi(_clk); ps2_delay_us(PS2_CLK_QUARTER_US);
  taskEXIT_CRITICAL(&mux);
  return 0;
}

int PS2Mouse::ps2_write_wait_idle(uint8_t data, uint64_t timeout_us) {
  uint64_t start = ps2_micros();
  while (get_bus_state() != BusState::IDLE) {
    if (ps2_micros() - start > timeout_us) return -1;
    vTaskDelay(1);
  }
  return ps2_write(data);
}

int PS2Mouse::ps2_read(uint8_t *value, uint64_t timeout_ms) {
  unsigned long wait_start = millis();
  while (get_bus_state() != BusState::HOST_REQUEST_TO_SEND) {
    if (timeout_ms > 0 && (millis() - wait_start) > timeout_ms) return -1;
    vTaskDelay(1);
  }
  uint8_t data = 0, bit = 1;
  uint8_t calc_parity = 1, recv_parity = 0;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  taskENTER_CRITICAL(&mux);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_clk); ps2_delay_us(PS2_CLK_HALF_US); _gohi(_clk); ps2_delay_us(PS2_CLK_QUARTER_US);
  for (int i = 0; i < 8; i++) {
    if (gpio_get_level((gpio_num_t)_dat)) { data |= bit; calc_parity ^= 1; }
    bit <<= 1;
    ps2_delay_us(PS2_CLK_QUARTER_US);
    _golo(_clk); ps2_delay_us(PS2_CLK_HALF_US); _gohi(_clk); ps2_delay_us(PS2_CLK_QUARTER_US);
  }
  if (gpio_get_level((gpio_num_t)_dat)) recv_parity = 1;
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_clk); ps2_delay_us(PS2_CLK_HALF_US); _gohi(_clk); ps2_delay_us(PS2_CLK_QUARTER_US);
  ps2_delay_us(PS2_CLK_QUARTER_US);
  _golo(_dat); _golo(_clk);
  ps2_delay_us(PS2_CLK_HALF_US);
  _gohi(_clk); ps2_delay_us(PS2_CLK_QUARTER_US); _gohi(_dat);
  taskEXIT_CRITICAL(&mux);
  *value = data;
  return (recv_parity == calc_parity) ? 0 : -2;
}

void PS2Mouse::_ack() {
  ps2_delay_us(PS2_BYTE_INTERVAL_US);
  ps2_write(0xFA);
  ps2_delay_us(PS2_BYTE_INTERVAL_US);
}

int PS2Mouse::send_packet(PS2Packet *pkt) {
  return (xQueueSend(_queue, pkt, 0) == pdTRUE) ? 0 : -1;
}

// ── PS/2 mouse host command handler ──────────────────────────────────────────
// Supports IntelliMouse detection (magic sample-rate sequence 200→100→80).
void PS2Mouse::reply_to_host(uint8_t cmd) {
  uint8_t val;
  switch (cmd) {
    case 0xFF: // Reset — disable reporting, send BAT + device ID
      _dataReporting = false;
      _intelliMouse  = false;
      _explorerMouse = false;
      memset(_sampleHistory, 0, 3);
      // Flush queued movement packets BEFORE sending BAT.
      // Host task holds the mutex here — send task cannot dequeue.
      // Prevents stale movement bytes from appearing after BAT during driver reload.
      flushQueue();
      _ack();
      vTaskDelay(pdMS_TO_TICKS(20));
      while (ps2_write(0xAA) != 0) vTaskDelay(1);
      ps2_delay_us(PS2_BYTE_INTERVAL_US);
      while (ps2_write(0x00) != 0) vTaskDelay(1); // Standard mouse ID
      break;

    case 0xFE: // Resend
      _ack();
      break;

    case 0xF6: // Set Defaults
      _dataReporting = false;
      _intelliMouse  = false;
      _explorerMouse = false;
      memset(_sampleHistory, 0, 3);
      _ack();
      break;

    case 0xF5: // Disable Data Reporting
      _dataReporting = false;
      _ack();
      break;

    case 0xF4: // Enable Data Reporting
      _dataReporting = true;
      _ack();
      break;

    case 0xF3: // Set Sample Rate — track last 3 for IntelliMouse detection
      _ack();
      if (ps2_read(&val) == 0) {
        _sampleHistory[0] = _sampleHistory[1];
        _sampleHistory[1] = _sampleHistory[2];
        _sampleHistory[2] = val;
        _ack();
      }
      break;

    case 0xF2: // Get Device ID
      // PS/2 mouse protocol negotiation via magic Set Sample Rate sequences:
      //
      //  Standard   (ID=0x00): default after Reset
      //  IntelliMouse (ID=0x03): host sends rates 200→100→80, then Get ID
      //  Explorer   (ID=0x04): after reaching 0x03, host sends 200→200→80, then Get ID
      //
      // g_ps2ProtoMode caps the maximum protocol level — host magic sequences
      // above that cap are silently ignored and current ID is returned unchanged.
      _ack();
      if (g_ps2ProtoMode >= 0x04 && _intelliMouse &&
          _sampleHistory[0] == 200 && _sampleHistory[1] == 200 && _sampleHistory[2] == 80) {
        // Stage 2: IntelliMouse Explorer (5 buttons + scroll)
        _explorerMouse = true;
        logQ_str("[PS2MOUSE] Explorer mode (ID=0x04) — Back/Forward + scroll\n");
        while (ps2_write(0x04) != 0) vTaskDelay(1);
      } else if (g_ps2ProtoMode >= 0x03 && !_explorerMouse &&
                 _sampleHistory[0] == 200 && _sampleHistory[1] == 100 && _sampleHistory[2] == 80) {
        // Stage 1: IntelliMouse (scroll wheel, 3 buttons)
        _intelliMouse = true;
        logQ_str("[PS2MOUSE] IntelliMouse mode (ID=0x03) — scroll wheel\n");
        while (ps2_write(0x03) != 0) vTaskDelay(1);
      } else {
        // No matching magic, or proto cap reached — return current ID
        uint8_t id = _explorerMouse ? 0x04 : (_intelliMouse ? 0x03 : 0x00);
        while (ps2_write(id) != 0) vTaskDelay(1);
      }
      break;

    case 0xEB: // Read Data (remote mode) — reply with one empty movement packet
      _ack();
      {
        // Explorer and IntelliMouse both use 4-byte packets
        uint8_t n = (_intelliMouse || _explorerMouse) ? 4 : 3;
        uint8_t buf[4] = { 0x08, 0x00, 0x00, 0x00 }; // bit3=1, no movement
        for (int i = 0; i < n; i++) {
          ps2_write_wait_idle(buf[i]);
          ps2_delay_us(PS2_BYTE_INTERVAL_US);
        }
      }
      break;

    case 0xEA: // Set Stream Mode
    case 0xF0: // Reserved / set remote mode in some specs
    case 0xE8: // Set Resolution — read and discard parameter byte
      _ack();
      if (cmd == 0xE8 && ps2_read(&val) == 0) _ack();
      break;

    case 0xE7: // Set Scaling 2:1
    case 0xE6: // Set Scaling 1:1
    case 0xEE: // Set Wrap Mode
    case 0xEC: // Reset Wrap Mode
    case 0xE9: // Status Request — answer with 3 status bytes (all zeros for simplicity)
      _ack();
      if (cmd == 0xE9) {
        ps2_write_wait_idle(0x00); ps2_delay_us(PS2_BYTE_INTERVAL_US); // status
        ps2_write_wait_idle(0x02); ps2_delay_us(PS2_BYTE_INTERVAL_US); // resolution=2
        ps2_write_wait_idle(0x64); ps2_delay_us(PS2_BYTE_INTERVAL_US); // sample rate=100
      }
      break;

    default:
      _ack();
      break;
  }
}

// ── PS/2 mouse — send movement packet ────────────────────────────────────────
// Protocol byte layout:
//
//  Standard (3B, ID=0x00):
//    [0]: [YO][XO][YS][XS][ 1][MB][RB][LB]
//    [1]: X delta (int8)
//    [2]: Y delta (int8, PS/2 +Y = UP, so negate BLE Y)
//
//  IntelliMouse (4B, ID=0x03):
//    [0..2]: same as standard
//    [3]: scroll wheel (int8, full range ±127)
//
//  Explorer (4B, ID=0x04):
//    [0..2]: same as standard
//    [3]: [0][0][B5][B4][W3][W2][W1][W0]
//          B4 = Back button (BLE bit3)
//          B5 = Forward button (BLE bit4)
//          W3..W0 = scroll wheel, 4-bit signed (-8..+7)
//
//  BLE button bits → PS/2:
//    bit0=L  bit1=R  bit2=M  → byte0 bits 0-2  (all modes)
//    bit3=Back  bit4=Fwd     → byte3 bits 4-5   (Explorer only)
void PS2Mouse::sendMovement(int8_t dx, int8_t dy_hid, int8_t dw, uint8_t btns) {
  if (!_dataReporting) return;

  int8_t dy = -dy_hid; // PS/2 Y: positive = cursor UP (opposite of HID)

  // Byte 0: status
  uint8_t status = 0x08; // bit3 always 1 per spec
  if (btns & 0x01) status |= 0x01; // Left
  if (btns & 0x02) status |= 0x02; // Right
  if (btns & 0x04) status |= 0x04; // Middle
  if (dx < 0)      status |= 0x10; // X sign
  if (dy < 0)      status |= 0x20; // Y sign

  PS2Packet pkt;
  pkt.data[0] = status;
  pkt.data[1] = (uint8_t)dx;
  pkt.data[2] = (uint8_t)dy;

  if (_explorerMouse) {
    // Byte 3: 4-bit signed wheel + B4 (Back) + B5 (Forward)
    // Wheel clamped to -8..+7, encoded as lower nibble 2's complement
    int8_t w4 = (int8_t)constrain((int)dw, -8, 7);
    uint8_t byte3 = (uint8_t)(w4 & 0x0F);         // bits 3:0 = wheel
    if (btns & 0x08) byte3 |= 0x10;               // bit4 = Back
    if (btns & 0x10) byte3 |= 0x20;               // bit5 = Forward
    pkt.data[3] = byte3;
    pkt.len = 4;
  } else if (_intelliMouse) {
    // Byte 3: full int8 scroll wheel, no button bits
    pkt.data[3] = (uint8_t)dw;
    pkt.len = 4;
  } else {
    pkt.len = 3; // scroll and extra buttons silently dropped
  }
  send_packet(&pkt);
}

// ── PS/2 mouse — begin() ─────────────────────────────────────────────────────
void PS2Mouse::begin() {
  gpio_config_t io = {};
  io.intr_type    = GPIO_INTR_DISABLE;
  io.mode         = GPIO_MODE_OUTPUT_OD;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.pull_up_en   = GPIO_PULLUP_DISABLE;
  io.pin_bit_mask = (1ULL << _clk); gpio_config(&io);
  io.pin_bit_mask = (1ULL << _dat); gpio_config(&io);
  _gohi(_clk); _gohi(_dat);

  _mutex = xSemaphoreCreateMutex();
  _queue = xQueueCreate(PS2_PACKET_QUEUE_LEN, sizeof(PS2Packet));

  // Send BAT then Device ID (0x00 = standard mouse)
  xSemaphoreTake(_mutex, portMAX_DELAY);
  ps2_delay_us(PS2_BYTE_INTERVAL_US);
  vTaskDelay(pdMS_TO_TICKS(200));
  ps2_write(0xAA);
  ps2_delay_us(PS2_BYTE_INTERVAL_US * 2);
  ps2_write(0x00); // Device ID: standard mouse (IntelliMouse activates later via host command)
  xSemaphoreGive(_mutex);

  xTaskCreatePinnedToCore(ps2mouse_task_host, "ps2mhost", 4096, this,  8, &_task_host, 0);
  // Core 0 — same reasoning as ps2kb_task_send: taskENTER_CRITICAL in ps2_write
  // must not run on Core 1 where UART RX interrupts live.
  xTaskCreatePinnedToCore(ps2mouse_task_send, "ps2msend", 4096, this, 15, &_task_send, 0);
}

void ps2mouse_task_host(void *arg) {
  PS2Mouse *m = (PS2Mouse *)arg;
  while (true) {
    xSemaphoreTake(m->get_mutex(), portMAX_DELAY);
    if (m->get_bus_state() == PS2Mouse::BusState::HOST_REQUEST_TO_SEND) {
      uint8_t cmd;
      if (m->ps2_read(&cmd) == 0) m->reply_to_host(cmd);
    }
    xSemaphoreGive(m->get_mutex());
    vTaskDelay(pdMS_TO_TICKS(PS2_HOST_POLL_MS));
  }
}

void ps2mouse_task_send(void *arg) {
  PS2Mouse *m = (PS2Mouse *)arg;
  while (true) {
    PS2Packet pkt;
    if (xQueueReceive(m->get_queue(), &pkt, portMAX_DELAY) == pdTRUE) {
      xSemaphoreTake(m->get_mutex(), portMAX_DELAY);
      ps2_delay_us(PS2_BYTE_INTERVAL_US);
      for (int i = 0; i < pkt.len; i++) {
        // Retry until sent — host may inhibit clock while reading previous byte.
        while (m->ps2_write_wait_idle(pkt.data[i]) != 0) {
          ps2_delay_us(PS2_BYTE_INTERVAL_US);
        }
        if (i < pkt.len - 1)
          ps2_delay_us(PS2_BYTE_INTERVAL_US);
      }
      xSemaphoreGive(m->get_mutex());
    }
  }
}

// ════════════════════════════════════════════════════════════════════════════════
//  MAIN SKETCH
// ════════════════════════════════════════════════════════════════════════════════

// ── Pin configuration ─────────────────────────────────────────────────────────
#define PS2_KB_CLK_PIN    19   // Keyboard PS/2 CLK  ←→ GPIO19
#define PS2_KB_DAT_PIN    18   // Keyboard PS/2 DATA ←→ GPIO18
#define PS2_MOUSE_CLK_PIN 16   // Mouse    PS/2 CLK  ←→ GPIO16
#define PS2_MOUSE_DAT_PIN 17   // Mouse    PS/2 DATA ←→ GPIO17

// ── Misc configuration ────────────────────────────────────────────────────────
#define BRIDGE_NAME   "BLE-PS2-Bridge"
#define NVS_NS        "ble-ps2"
// Both keyboard and mouse use up-to-20 attempt retry loops in handleSerial()

// ── Typematic ─────────────────────────────────────────────────────────────────
#define TYPEMATIC_DELAY_MS 500
#define TYPEMATIC_RATE_MS   50
#define KEEPALIVE_MS      30000   // battery keepalive — every 30 s is enough

// ── PS/2 device instances ─────────────────────────────────────────────────────
static PS2Keyboard keyboard(PS2_KB_CLK_PIN, PS2_KB_DAT_PIN);
static PS2Mouse    mouse(PS2_MOUSE_CLK_PIN, PS2_MOUSE_DAT_PIN);

// ── BLE UUIDs ─────────────────────────────────────────────────────────────────
static BLEUUID HID_SVC_UUID ("00001812-0000-1000-8000-00805f9b34fb");
static BLEUUID HID_RPT_UUID ("00002a4d-0000-1000-8000-00805f9b34fb");
static BLEUUID BAT_SVC_UUID ("0000180f-0000-1000-8000-00805f9b34fb");
static BLEUUID BAT_LVL_UUID ("00002a19-0000-1000-8000-00805f9b34fb");
static BLEUUID RPT_REF_UUID ("00002908-0000-1000-8000-00805f9b34fb");

// ── Keyboard BLE state ────────────────────────────────────────────────────────
static NimBLEClient* pClientKb          = nullptr;
static bool          kbReconnectScan    = false;
static unsigned long kbReconnectAt      = 0;
static int           kbReconnectFails   = 0;
static char          kbMAC[18]          = "";
static uint8_t       kbType             = 1;
static int           kbBattery          = -1;
static unsigned long kbKeepaliveAt      = 0;
static NimBLERemoteCharacteristic* pKbLedChar = nullptr;
static NimBLERemoteCharacteristic* pKbBatChar = nullptr;
static volatile bool kbConnecting       = false; // true while tryConnectKb() is running

// Keyboard HID state
static uint8_t prevKeys[6] = {0};
static uint8_t prevMod     = 0;

// Typematic
// volatile — written by processHIDReport (Core 0 BLE callback), read by loop() (Core 1)
static volatile unsigned long typematicNext  = 0;
static volatile bool          typematicArmed = false;
static volatile uint8_t       typematicKey   = 0;

// Hotkey flags
static volatile bool statusRequested  = false;
static volatile bool batteryRequested = false;

// ── Mouse BLE state ───────────────────────────────────────────────────────────
static NimBLEClient* pClientMouse        = nullptr;
static bool          mouseReconnectScan  = false;
static unsigned long mouseReconnectAt    = 0;
static int           mouseReconnectFails = 0;
static char          mouseMAC[18]        = "";
static uint8_t       mouseType           = 1;
static int           mouseBattery        = -1;
static unsigned long mouseKeepaliveAt    = 0;
static NimBLERemoteCharacteristic* pMouseBatChar = nullptr;
static volatile bool mouseConnecting     = false; // true while tryConnectMouse() is running

// Mouse settings (saved to NVS)
static volatile int     g_scaleDivisor   = 4;
static volatile bool    g_flipY          = false;
static volatile bool    g_flipW          = false;
static volatile uint8_t g_filterReportId = 0;
// PS/2 protocol mode cap — maximum ID the firmware will negotiate to.
//   0x00 = Standard (3-byte, no scroll, no extra buttons)
//   0x03 = IntelliMouse (4-byte, scroll wheel)
//   0x04 = IntelliMouse Explorer (4-byte, scroll + Back + Forward)  ← default
volatile uint8_t g_ps2ProtoMode   = 0x04;

static const char* ps2ProtoName(uint8_t m) {
  switch (m) {
    case 0x00: return "Standard (0x00, 3-btn no scroll)";
    case 0x03: return "IntelliMouse (0x03, scroll wheel)";
    case 0x04: return "Explorer (0x04, scroll+Back+Fwd)";
    default:   return "unknown";
  }
}

// Mouse movement accumulator (BLE callback → processMouseMovement in loop)
static portMUX_TYPE   g_mux       = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t g_accX    = 0;
static volatile int32_t g_accY    = 0;
static volatile int32_t g_accW    = 0;
static volatile uint8_t g_buttons = 0; // bit0=L bit1=R bit2=M bit3=Back bit4=Fwd
static volatile bool    g_dirty   = false;
static uint8_t          g_prevButtons = 0;

// Mouse report handle filter (subscribe only to specific report IDs)
#define MAX_MOUSE_HANDLES 8
static uint16_t g_mouseHandles[MAX_MOUSE_HANDLES];
static int      g_mouseHandleCount = 0;
static bool isMouseHandle(uint16_t h) {
  for (int i = 0; i < g_mouseHandleCount; i++)
    if (g_mouseHandles[i] == h) return true;
  return false;
}

// Debug throttle

// ── Scan state ────────────────────────────────────────────────────────────────
static int                  scanCount    = 0;
static unsigned long        scanEndAt    = 0;
static volatile bool        isManualScan = false;
static std::set<std::string> scanSeen;

static Preferences prefs;

// ════════════════════════════════════════════════════════════════════════════════
//  MOUSE — BLE HID report parsing
// ════════════════════════════════════════════════════════════════════════════════
//
//  Supported BLE mouse payload formats (WITHOUT Report ID byte):
//   [3B]  btn | X8 | Y8
//   [4B]  btn | X8 | Y8 | wheel
//   [5B]  btn | X8 | Y8 | wheel | hwheel
//   [7B]  Logitech 12-bit packed (MX Master 2/3, G502, M650, etc.)
//         btn | extra | X_lo | X_hi|Y_lo | Y_hi | wheel | hwheel
//
//  Button bit mapping in g_buttons:
//   bit0=Left  bit1=Right  bit2=Middle  bit3=Back  bit4=Forward

static inline int16_t sign12(int32_t v) {
  return (v & 0x800) ? (int16_t)(v - 0x1000) : (int16_t)v;
}

static bool parseBLEMouseReport(const uint8_t* d, size_t len,
                                 int16_t& dx, int16_t& dy,
                                 int8_t& wheel, uint8_t& buttons) {
  if (len < 3 || len > 10) return false;
  if (d[0] == 0xFF) return false; // Logitech vendor specific frame
  wheel = 0;
  if (len <= 5) {
    buttons = d[0] & 0x07;
    dx = (int8_t)d[1]; dy = (int8_t)d[2];
    if (len >= 4) wheel = (int8_t)d[3];
    return true;
  }
  if (len == 7) {
    // Logitech 12-bit packed
    buttons = d[0] & 0x1F;
    if (d[1] & 0xFF) buttons |= 0x20; // extra button → bit5
    dx    = sign12((int32_t)d[2] | (((int32_t)d[3] & 0x0F) << 8));
    dy    = sign12(((int32_t)d[3] >> 4) | ((int32_t)d[4] << 4));
    wheel = (int8_t)d[5];
    return true;
  }
  // Fallback 16-bit XY
  buttons = d[0] & 0x07;
  dx = (int16_t)((uint16_t)d[1] | ((uint16_t)d[2] << 8));
  dy = (int16_t)((uint16_t)d[3] | ((uint16_t)d[4] << 8));
  return true;
}

static void mouseNotifyCallback(NimBLERemoteCharacteristic* pChar,
                                 uint8_t* pData, size_t length, bool isNotify) {
  if (g_mouseHandleCount > 0 && !isMouseHandle(pChar->getHandle())) return;

  int16_t dx = 0, dy = 0; int8_t dw = 0; uint8_t btns = 0;
  if (!parseBLEMouseReport(pData, length, dx, dy, dw, btns)) {
    // Unknown report format — silently ignore.
    // No Serial here: callback runs on Core 0, Serial races with loop() on Core 1.
    return;
  }
  portENTER_CRITICAL(&g_mux);
    uint8_t prev = g_buttons;
    g_accX += dx; g_accY += dy; g_accW += dw;
    g_buttons = btns; g_dirty = true;
  portEXIT_CRITICAL(&g_mux);
  // Button change logging moved to processMouseMovement() which runs on Core 1
  (void)prev;
}

// ── Mouse movement → PS/2 ────────────────────────────────────────────────────
// Called from loop(). Drains the BLE accumulator and sends PS/2 mouse packets.
void processMouseMovement() {
  bool buttonChanged = (g_buttons != g_prevButtons);
  if (!g_dirty && !buttonChanged) return;

  int32_t ax, ay, aw;
  uint8_t btns;
  portENTER_CRITICAL(&g_mux);
    ax = g_accX; ay = g_accY; aw = g_accW;
    btns = g_buttons; g_dirty = false;
  portEXIT_CRITICAL(&g_mux);

  int div = (g_scaleDivisor < 1) ? 1 : g_scaleDivisor;
  int32_t sx = ax / div, remX = ax - sx * div;
  int32_t sy = ay / div, remY = ay - sy * div;
  int32_t sw = aw; // wheel not scaled

  portENTER_CRITICAL(&g_mux);
    g_accX = remX; g_accY = remY; g_accW = 0;
  portEXIT_CRITICAL(&g_mux);

  if (g_flipY) sy = -sy;
  if (g_flipW) sw = -sw;

  if (sx == 0 && sy == 0 && sw == 0 && btns == g_prevButtons) return;

  // Button change log — throttled, Core 1, non-blocking via logQ
  if (btns != g_prevButtons)
    logQ("[BTN] L=%d R=%d M=%d Bk=%d Fw=%d\n",
      btns&1, (btns>>1)&1, (btns>>2)&1, (btns>>3)&1, (btns>>4)&1);

  g_prevButtons = btns;

  // Send movement in ±127 chunks (PS/2 mouse byte is int8_t)
  int32_t totalX=0, totalY=0, totalW=0; uint32_t pkts=0;
  do {
    int8_t px = (int8_t)constrain(sx, -127, 127);
    int8_t py = (int8_t)constrain(sy, -127, 127);
    int8_t pw = (int8_t)constrain(sw, -127, 127);
    mouse.sendMovement(px, py, pw, btns);
    totalX+=px; totalY+=py; totalW+=pw; pkts++;
    sx -= px; sy -= py; sw -= pw;
    if (!sx && !sy && !sw) break;
  } while (sx || sy || sw);

  // Throttled movement summary — max once per 500 ms
  static unsigned long _lastMovePrint = 0;
  static int32_t _sumX=0, _sumY=0, _sumW=0; static uint32_t _sumPkts=0;
  _sumX+=totalX; _sumY+=totalY; _sumW+=totalW; _sumPkts+=pkts;
  unsigned long now = millis();
  if (now - _lastMovePrint >= 500 && _sumPkts > 0) {
    logQ("[MOUSE] pkts=%lu X=%d Y=%d W=%d btn=L%d R%d M%d Bk%d Fw%d\n",
      _sumPkts, (int)_sumX, (int)_sumY, (int)_sumW,
      btns&1, (btns>>1)&1, (btns>>2)&1, (btns>>3)&1, (btns>>4)&1);
    _sumX=_sumY=_sumW=0; _sumPkts=0; _lastMovePrint=now;
  }
}

// ════════════════════════════════════════════════════════════════════════════════
//  KEYBOARD — BLE HID report processing  (identical to v1.6)
// ════════════════════════════════════════════════════════════════════════════════

void processHIDReport(uint8_t* data, size_t len) {
  if (len < 2) return;
  uint8_t mod = data[0];
  bool hasReserved = (len == 8 && data[1] == 0x00);
  uint8_t* keys  = hasReserved ? data + 2 : data + 1;
  size_t   nkeys = hasReserved ? len - 2  : len - 1;
  if (nkeys > 6) nkeys = 6;

  // Raw HID dump — via logQ (Core 0 safe, non-blocking)
  {
    char _t[64]; int _n = snprintf(_t, sizeof(_t), "[KB]");
    for (size_t i = 0; i < len && _n < 60; i++)
      _n += snprintf(_t+_n, sizeof(_t)-_n, " %02X", data[i]);
    snprintf(_t+_n, sizeof(_t)-_n, "\n");
    logQ_str(_t);
  }

  // Modifier keys
  for (int bit = 0; bit < 8; bit++) {
    uint8_t mask = 1 << bit;
    bool wasDown = (prevMod & mask) != 0;
    bool isDown  = (mod     & mask) != 0;
    if ( isDown && !wasDown) keyboard.keyHid_send((uint8_t)(0xE0 + bit), true);
    if (!isDown &&  wasDown) keyboard.keyHid_send((uint8_t)(0xE0 + bit), false);
  }
  // Released keys
  for (int i = 0; i < 6; i++) {
    if (prevKeys[i] == 0 || prevKeys[i] == 0x01) continue;
    bool found = false;
    for (size_t j = 0; j < nkeys; j++) if (keys[j] == prevKeys[i]) { found=true; break; }
    if (!found) keyboard.keyHid_send(prevKeys[i], false);
  }
  // Pressed keys
  for (size_t i = 0; i < nkeys; i++) {
    if (keys[i] == 0 || keys[i] == 0x01) continue;
    bool found = false;
    for (int j = 0; j < 6; j++) if (prevKeys[j] == keys[i]) { found=true; break; }
    if (!found) keyboard.keyHid_send(keys[i], true);
  }

  prevMod = mod;
  memset(prevKeys, 0, 6);
  memcpy(prevKeys, keys, nkeys);

  // Hotkey detection
  bool lctrl  = (mod & 0x01) != 0;
  bool lalt   = (mod & 0x04) != 0;
  bool lshift = (mod & 0x02) != 0;
  bool prtsc  = false;
  for (size_t i = 0; i < nkeys; i++) if (keys[i]==0x46||keys[i]==0x9A) { prtsc=true; break; }
  if (lctrl && lalt && lshift && prtsc)  statusRequested  = true;
  if (lctrl && lalt && !lshift && prtsc) batteryRequested = true;

  // Typematic
  typematicKey = 0;
  for (size_t i = 0; i < nkeys; i++) {
    if (keys[i] && keys[i]!=0x01 && keys[i]!=0x39) { typematicKey=keys[i]; break; }
  }
  if (typematicKey) { typematicArmed=false; typematicNext=millis()+TYPEMATIC_DELAY_MS; }
  else              { typematicArmed=false; typematicNext=0; }
}

void kbNotifyCallback(NimBLERemoteCharacteristic* pChar,
                      uint8_t* pData, size_t length, bool isNotify) {
  processHIDReport(pData, length);
}

// ════════════════════════════════════════════════════════════════════════════════
//  BLE SCAN CALLBACKS
// ════════════════════════════════════════════════════════════════════════════════

static const char* bleAppName(uint16_t a) {
  switch (a) {
    case 0x03C1: return "Keyboard";
    case 0x03C2: return "Mouse";
    case 0x03C3: return "Joystick";
    case 0x03C4: return "Gamepad";
    default:     return "HID";
  }
}

class MyScanCallbacks : public NimBLEScanCallbacks {
  void onDiscovered(const NimBLEAdvertisedDevice* dev) override {
    std::string addr = dev->getAddress().toString();
    // Reconnect scans — check saved MACs
    if (kbReconnectScan && strlen(kbMAC) && addr == std::string(kbMAC)) {
      logQ("[SCAN] Keyboard found — connecting...\n");
      NimBLEDevice::getScan()->stop();
      kbReconnectScan = false; kbReconnectAt = millis() + 50;
    }
    if (mouseReconnectScan && strlen(mouseMAC) && addr == std::string(mouseMAC)) {
      logQ("[SCAN] Mouse found — connecting...\n");
      NimBLEDevice::getScan()->stop();
      mouseReconnectScan = false; mouseReconnectAt = millis() + 50;
    }
  }

  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (!isManualScan) return;
    if (!dev->haveServiceUUID() || !dev->isAdvertisingService(HID_SVC_UUID)) return;
    std::string addr = dev->getAddress().toString();
    if (scanSeen.count(addr)) return;
    scanSeen.insert(addr);
    const char* app = dev->haveAppearance() ? bleAppName(dev->getAppearance()) : "HID";
    std::string nameStr = (dev->haveName() && dev->getName().length()) ? dev->getName() : "-";
    logQ("  #%-2d  %-17s  %-10s  %4d dBm  %s\n",
      scanCount+1, addr.c_str(), app, dev->getRSSI(), nameStr.c_str());
    scanCount++;
  }

  void onScanEnd(const NimBLEScanResults&, int) override {
    if (kbReconnectScan)    { kbReconnectScan    = false; kbReconnectAt    = millis() + 200; }
    if (mouseReconnectScan) { mouseReconnectScan = false; mouseReconnectAt = millis() + 200; }
  }
};

// ════════════════════════════════════════════════════════════════════════════════
//  BLE CONNECT — keyboard  (same logic as v1.6 tryConnect, renamed)
// ════════════════════════════════════════════════════════════════════════════════

bool tryConnectKb(NimBLEAddress addr) {
  if (pClientKb) {
    if (pClientKb->isConnected()) pClientKb->disconnect();
    NimBLEDevice::deleteClient(pClientKb); pClientKb = nullptr; delay(200);
  }
  Serial.printf("[KB] Connecting to %s ...\n", addr.toString().c_str());
  pClientKb = NimBLEDevice::createClient();
  pClientKb->setConnectionParams(6, 6, 0, 3200);  // fixed 7.5 ms interval — lowest BLE latency
  pClientKb->setConnectTimeout(12);
  if (!pClientKb->connect(addr)) {
    NimBLEDevice::deleteClient(pClientKb); pClientKb = nullptr; return false;
  }
  if (pClientKb->secureConnection()) Serial.println("[KB] Bonding OK");

  NimBLERemoteService* svc = pClientKb->getService(HID_SVC_UUID);
  if (!svc) { pClientKb->disconnect(); NimBLEDevice::deleteClient(pClientKb); pClientKb=nullptr; return false; }

  int subs = 0;
  const std::vector<NimBLERemoteCharacteristic*>& chars = svc->getCharacteristics(&HID_RPT_UUID);
  for (NimBLERemoteCharacteristic* c : chars)
    if (c->canNotify()) { c->subscribe(true, kbNotifyCallback); subs++; }

  if (!subs) { pClientKb->disconnect(); NimBLEDevice::deleteClient(pClientKb); pClientKb=nullptr; return false; }

  pKbLedChar = nullptr;
  for (NimBLERemoteCharacteristic* c : chars)
    if (c->canWrite()) { pKbLedChar = c; break; }
  if (pKbLedChar) Serial.println("[KB] LED output char found");

  kbBattery = -1; pKbBatChar = nullptr;
  NimBLERemoteService* bs = pClientKb->getService(BAT_SVC_UUID);
  if (bs) {
    NimBLERemoteCharacteristic* bc = bs->getCharacteristic(BAT_LVL_UUID);
    if (bc) {
      if (bc->canRead()) { NimBLEAttValue v=bc->readValue(); if(v.size()>0) kbBattery=(int)(uint8_t)v[0]; pKbBatChar=bc; }
      // Subscribe: device pushes the actual current battery shortly after subscribe,
      // correcting any stale readValue() result from connect time.
      if (bc->canNotify()) bc->subscribe(true, [](NimBLERemoteCharacteristic*, uint8_t* d, size_t l, bool) { if(l>0) kbBattery=(int)d[0]; });
    }
  }
  memset(prevKeys, 0, 6); prevMod = 0;
  kbKeepaliveAt = millis() + KEEPALIVE_MS;
  Serial.printf("[KB] Connected — %d char(s), battery %d%%\n", subs, kbBattery);
  return true;
}

// ════════════════════════════════════════════════════════════════════════════════
//  BLE CONNECT — mouse  (from ble_usb_kb_mouse_bridge_s3, adapted for PS/2)
// ════════════════════════════════════════════════════════════════════════════════

bool tryConnectMouse(NimBLEAddress addr) {
  if (pClientMouse) {
    if (pClientMouse->isConnected()) pClientMouse->disconnect();
    NimBLEDevice::deleteClient(pClientMouse); pClientMouse = nullptr; delay(200);
  }
  Serial.printf("[MOUSE] Connecting to %s ...\n", addr.toString().c_str());
  pClientMouse = NimBLEDevice::createClient();
  pClientMouse->setConnectionParams(6, 6, 0, 3200);  // fixed 7.5 ms interval
  pClientMouse->setConnectTimeout(12);
  if (!pClientMouse->connect(addr)) {
    NimBLEDevice::deleteClient(pClientMouse); pClientMouse = nullptr; return false;
  }
  if (pClientMouse->secureConnection()) Serial.println("[MOUSE] Bonding OK");

  NimBLERemoteService* svc = pClientMouse->getService(HID_SVC_UUID);
  if (!svc) { pClientMouse->disconnect(); NimBLEDevice::deleteClient(pClientMouse); pClientMouse=nullptr; return false; }

  // Use Report Reference descriptor (0x2908) to subscribe only to Input reports
  g_mouseHandleCount = 0;
  int subs = 0;
  const std::vector<NimBLERemoteCharacteristic*>& chars = svc->getCharacteristics(&HID_RPT_UUID);
  for (NimBLERemoteCharacteristic* c : chars) {
    if (!c->canNotify()) continue;
    uint8_t rptId = 0xFF, rptType = 1;
    NimBLERemoteDescriptor* ref = c->getDescriptor(RPT_REF_UUID);
    if (ref) {
      std::string v = ref->readValue();
      if (v.size() >= 2) { rptId = v[0]; rptType = v[1]; }
    }
    bool skip = (rptType != 1) || (rptId == 0) ||
                (g_filterReportId && rptId != g_filterReportId);
    Serial.printf("[MOUSE] handle=0x%04X  ID=%-3d  Type=%d  %s\n",
      c->getHandle(), rptId, rptType, skip ? "skip" : "SUBSCRIBE");
    if (skip) continue;
    c->subscribe(true, mouseNotifyCallback); subs++;
    if (g_mouseHandleCount < MAX_MOUSE_HANDLES)
      g_mouseHandles[g_mouseHandleCount++] = c->getHandle();
  }
  // Fallback: subscribe all if all were filtered out
  if (!subs) {
    Serial.println("[MOUSE] Fallback: subscribing all notifiable chars");
    for (NimBLERemoteCharacteristic* c : chars) {
      if (!c->canNotify()) continue;
      c->subscribe(true, mouseNotifyCallback); subs++;
      if (g_mouseHandleCount < MAX_MOUSE_HANDLES)
        g_mouseHandles[g_mouseHandleCount++] = c->getHandle();
    }
  }
  if (!subs) { pClientMouse->disconnect(); NimBLEDevice::deleteClient(pClientMouse); pClientMouse=nullptr; return false; }

  mouseBattery = -1; pMouseBatChar = nullptr;
  NimBLERemoteService* bs = pClientMouse->getService(BAT_SVC_UUID);
  if (bs) {
    NimBLERemoteCharacteristic* bc = bs->getCharacteristic(BAT_LVL_UUID);
    if (bc) {
      if (bc->canRead()) { NimBLEAttValue v=bc->readValue(); if(v.size()>0) mouseBattery=(int)(uint8_t)v[0]; pMouseBatChar=bc; }
      if (bc->canNotify()) bc->subscribe(true, [](NimBLERemoteCharacteristic*, uint8_t* d, size_t l, bool) { if(l>0) mouseBattery=(int)d[0]; });
    }
  }
  mouseKeepaliveAt = millis() + KEEPALIVE_MS;
  portENTER_CRITICAL(&g_mux);
    g_accX=g_accY=g_accW=0; g_buttons=0; g_dirty=false;
  portEXIT_CRITICAL(&g_mux);
  g_prevButtons = 0;
  Serial.printf("[MOUSE] Connected — %d char(s), battery %d%%\n", subs, mouseBattery);
  return true;
}

// ════════════════════════════════════════════════════════════════════════════════
//  NVS — load / save
// ════════════════════════════════════════════════════════════════════════════════

static void loadSavedDevices() {
  prefs.begin(NVS_NS, true);
  String km = prefs.getString("kb-mac",  "");  kbType    = prefs.getUChar("kb-type",  1);
  String mm = prefs.getString("ms-mac",  "");  mouseType = prefs.getUChar("ms-type",  1);
  prefs.end();
  if (km.length()) { strncpy(kbMAC,    km.c_str(), 17); kbMAC[17]    = 0; }
  if (mm.length()) { strncpy(mouseMAC, mm.c_str(), 17); mouseMAC[17] = 0; }
}

static void saveKb(const NimBLEAddress& a) {
  strncpy(kbMAC, a.toString().c_str(), 17); kbMAC[17] = 0; kbType = (uint8_t)a.getType();
  prefs.begin(NVS_NS, false); prefs.putString("kb-mac", kbMAC); prefs.putUChar("kb-type", kbType); prefs.end();
  Serial.printf("[NVS] Keyboard saved: %s\n", kbMAC);
}

static void saveMouse(const NimBLEAddress& a) {
  strncpy(mouseMAC, a.toString().c_str(), 17); mouseMAC[17] = 0; mouseType = (uint8_t)a.getType();
  prefs.begin(NVS_NS, false); prefs.putString("ms-mac", mouseMAC); prefs.putUChar("ms-type", mouseType); prefs.end();
  Serial.printf("[NVS] Mouse saved: %s\n", mouseMAC);
}

static void saveMouseSettings() {
  prefs.begin(NVS_NS, false);
  prefs.putInt  ("ms-scale", g_scaleDivisor);
  prefs.putBool ("ms-flipy", g_flipY);
  prefs.putBool ("ms-flipw", g_flipW);
  prefs.putUChar("ms-rid",   g_filterReportId);
  prefs.putUChar("ms-proto", g_ps2ProtoMode);
  prefs.end();
}

static void loadMouseSettings() {
  prefs.begin(NVS_NS, true);
  g_scaleDivisor   = prefs.getInt  ("ms-scale", 4);
  g_flipY          = prefs.getBool ("ms-flipy", false);
  g_flipW          = prefs.getBool ("ms-flipw", false);
  g_filterReportId = prefs.getUChar("ms-rid",   0);
  g_ps2ProtoMode   = prefs.getUChar("ms-proto", 0x04); // default = Explorer
  prefs.end();
  if (g_scaleDivisor < 1 || g_scaleDivisor > 64) g_scaleDivisor = 4;
  if (g_ps2ProtoMode != 0x00 && g_ps2ProtoMode != 0x03 && g_ps2ProtoMode != 0x04)
    g_ps2ProtoMode = 0x04; // sanitize — reject any corrupted value
}

// ════════════════════════════════════════════════════════════════════════════════
//  STATUS
// ════════════════════════════════════════════════════════════════════════════════

static String buildStatus() {
  bool kbConn    = pClientKb    && pClientKb->isConnected();
  bool mouseConn = pClientMouse && pClientMouse->isConnected();
  String s = "\n--- BLE-PS2 Bridge status ---\n";

  s += "--- Keyboard (PS/2 port 1) ---\n";
  s += String("BLE:      ") + (kbConn ? "CONNECTED" : "disconnected") + "\n";
  s += String("MAC:      ") + (strlen(kbMAC) ? kbMAC : "(none)") + "\n";
  if (kbConn) {
    s += String("RSSI:     ") + pClientKb->getRssi() + " dBm\n";
    s += String("Battery:  ") + (kbBattery >= 0 ? (String(kbBattery)+"%") : "unknown") + "\n";
  } else if (kbReconnectScan) s += "Scanning for keyboard...\n";

  s += "\n--- Mouse (PS/2 port 2) ---\n";
  s += String("BLE:      ") + (mouseConn ? "CONNECTED" : "disconnected") + "\n";
  s += String("MAC:      ") + (strlen(mouseMAC) ? mouseMAC : "(none)") + "\n";
  if (mouseConn) {
    s += String("RSSI:     ") + pClientMouse->getRssi() + " dBm\n";
    s += String("Battery:  ") + (mouseBattery >= 0 ? (String(mouseBattery)+"%") : "unknown") + "\n";
  } else if (mouseReconnectScan) s += "Scanning for mouse...\n";
  s += String("Scale:    1/") + g_scaleDivisor + "\n";
  s += String("FlipY:    ") + (g_flipY ? "yes" : "no") + "\n";
  s += String("FlipW:    ") + (g_flipW ? "yes" : "no") + "\n";
  s += String("ReportID: ") + g_filterReportId + (g_filterReportId ? "" : " (auto)") + "\n";
  s += String("Proto:    ") + ps2ProtoName(g_ps2ProtoMode) + "\n";
  s += "-----------------------------\n";
  return s;
}

// ── Type-out via PS/2 ─────────────────────────────────────────────────────────
// Used to type battery% and status text into the host PC via the PS/2 keyboard

static void ps2TypeKey(uint8_t hid, bool shift = false) {
  if (shift) keyboard.keyHid_send(0xE1, true);
  keyboard.keyHid_send(hid, true); delay(20);
  keyboard.keyHid_send(hid, false);
  if (shift) keyboard.keyHid_send(0xE1, false);
  delay(30);
}

void typeBatteryViaPS2() {
  // "K: 80% M: 50%"
  String kb = (kbBattery    >= 0) ? String(kbBattery)    + "%" : "?%";
  String ms = (mouseBattery >= 0) ? String(mouseBattery) + "%" : "?%";
  String s = "K: " + kb + " M: " + ms;
  for (int i = 0; i < (int)s.length(); i++) {
    char c = s[i];
    if      (c >= 'A' && c <= 'Z') ps2TypeKey((uint8_t)(0x04 + (c - 'A')), true);
    else if (c >= 'a' && c <= 'z') ps2TypeKey((uint8_t)(0x04 + (c - 'a')));
    else if (c >= '1' && c <= '9') ps2TypeKey((uint8_t)(0x1E + (c - '1')));
    else if (c == '0') ps2TypeKey(0x27);
    else if (c == '%') ps2TypeKey(0x22, true); // Shift+5
    else if (c == ':') ps2TypeKey(0x33, true); // Shift+;
    else if (c == ' ') ps2TypeKey(0x2C);
    else if (c == '?') ps2TypeKey(0x38, true); // Shift+/
  }
}

void typeStatusViaPS2() {
  String s = buildStatus();
  for (int i = 0; i < (int)s.length(); i++) {
    char c = s[i];
    if      (c == '\n') ps2TypeKey(0x28);                // Enter
    else if (c == ' ')  ps2TypeKey(0x2C);                // Space
    else if (c == '-')  ps2TypeKey(0x2D);                // Minus (US layout)
    else if (c == '/')  ps2TypeKey(0x38);                // Slash
    else if (c == '.')  ps2TypeKey(0x37);                // Period
    else if (c == ':')  ps2TypeKey(0x33, true);          // Shift+; = :
    else if (c == '%')  ps2TypeKey(0x22, true);          // Shift+5 = %
    else if (c == '(')  ps2TypeKey(0x26, true);          // Shift+9 = (
    else if (c == ')')  ps2TypeKey(0x27, true);          // Shift+0 = )
    else if (c == ',')  ps2TypeKey(0x36);                // Comma
    else if (c == '+')  ps2TypeKey(0x2E, true);          // Shift+= = +
    else if (c == '_')  ps2TypeKey(0x2D, true);          // Shift+- = _
    else if (c >= 'a' && c <= 'z') ps2TypeKey((uint8_t)(0x04 + (c - 'a')));
    else if (c >= 'A' && c <= 'Z') ps2TypeKey((uint8_t)(0x04 + (c - 'A')), true);
    else if (c >= '1' && c <= '9') ps2TypeKey((uint8_t)(0x1E + (c - '1')));
    else if (c == '0') ps2TypeKey(0x27);
    // other characters (e.g. unprintable) silently skipped
  }
}

// ════════════════════════════════════════════════════════════════════════════════
//  SERIAL CONSOLE
// ════════════════════════════════════════════════════════════════════════════════

void printHelp() {
  Serial.println("\n========================================");
  Serial.println("  BLE -> PS/2 Keyboard + Mouse Bridge");
  Serial.println("  ESP32-WROOM  v2.1");
  Serial.println("========================================");
  Serial.printf ("  KB  CLK=GPIO%d  DAT=GPIO%d\n", PS2_KB_CLK_PIN, PS2_KB_DAT_PIN);
  Serial.printf ("  MOUSE CLK=GPIO%d  DAT=GPIO%d\n", PS2_MOUSE_CLK_PIN, PS2_MOUSE_DAT_PIN);
  Serial.println("----------------------------------------");
  Serial.println("  scan                  Scan BLE HID devices (10s)");
  Serial.println("  connect kb <mac>      Connect BLE keyboard");
  Serial.println("  connect mouse <mac>   Connect BLE mouse");
  Serial.println("  forget kb             Forget keyboard");
  Serial.println("  forget mouse          Forget mouse");
  Serial.println("  forget all            Forget both + reset settings");
  Serial.println("  scale <1-64>          Mouse movement divisor (default 4)");
  Serial.println("  flipy                 Toggle mouse Y inversion");
  Serial.println("  flipw                 Toggle scroll inversion");
  Serial.println("  reportid <0-255>      Mouse Report ID filter (0=auto)");
  Serial.println("  proto <0|3|4>         PS/2 mouse protocol cap (default 4)");
  Serial.println("                          0 = Standard (3-btn, no scroll)");
  Serial.println("                          3 = IntelliMouse (scroll wheel)");
  Serial.println("                          4 = Explorer (scroll+Back+Fwd)");
  Serial.println("  status                Show connection status");
  Serial.println("  help                  Show this list");
  Serial.println("  Hotkeys via BLE keyboard:");
  Serial.println("    LCtrl+LAlt+PrtSc          Type KB+Mouse battery%");
  Serial.println("    LCtrl+LAlt+LShift+PrtSc   Type full status");
  Serial.println("========================================\n");
}

// ── Non-blocking serial line accumulator ─────────────────────────────────────
// Reads all available bytes without blocking. Returns true and sets `line`
// when a complete line is received (\n or \r or \r\n — all handled).
// Called once per loop() iteration; never holds loop() for more than the time
// it takes to drain whatever is already in the UART RX FIFO (~128 bytes).
static String _serialBuf = "";

static bool serialReadLine(String& line) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      // Accept \r, \n, or \r\n as line terminator
      _serialBuf.trim();
      if (_serialBuf.length() > 0) {
        line = _serialBuf;
        _serialBuf = "";
        return true;
      }
      // bare \r or \n with empty buffer — ignore (e.g. second byte of \r\n)
    } else {
      _serialBuf += c;
    }
  }
  return false;
}

void handleSerial() {
  String line;
  if (!serialReadLine(line)) return;
  Serial.printf("> %s\n", line.c_str()); // echo — confirms ESP32 received the command

  if (line == "help" || line == "?") {
    printHelp();

  } else if (line == "scan") {
    if (scanEndAt) { Serial.println("[SCAN] Already running."); return; }
    NimBLEDevice::getScan()->stop(); delay(100);
    kbReconnectScan = mouseReconnectScan = false;
    isManualScan = true; scanCount = 0; scanSeen.clear();
    NimBLEDevice::getScan()->start(0, false);
    scanEndAt = millis() + 10000;
    Serial.println("[SCAN] 10s — HID devices only (put devices into pairing mode)");
    Serial.println("  #    MAC                Type        RSSI    Name");
    Serial.println("  -------------------------------------------------");

  } else if (line.startsWith("connect kb ")) {
    String mac = line.substring(11); mac.trim();
    if (mac.length() < 17) { Serial.println("[ERR] connect kb xx:xx:xx:xx:xx:xx"); return; }
    if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt=0; isManualScan=false; }
    kbReconnectAt=0; kbReconnectFails=0;
    { NimBLEAddress a(mac.c_str(), 1); NimBLEDevice::deleteBond(a); } delay(100);
    NimBLEAddress addr(mac.c_str(), 1); bool ok=false; int attempt=0;
    Serial.println("[KB] Connecting — keep keyboard in pairing mode...");
    while (!ok) {
      attempt++;
      Serial.printf("[KB] Attempt %d\n", attempt);
      ok = tryConnectKb(addr);
      if (!ok) {
        if (attempt >= 20) { Serial.println("[KB] Giving up. Try: connect kb <mac> again."); break; }
        if (Serial.available()) { while(Serial.available()) Serial.read(); _serialBuf=""; Serial.println("[KB] Aborted."); break; }
        delay(500);
      }
    }
    if (ok) saveKb(pClientKb->getPeerAddress());

  } else if (line.startsWith("connect mouse ")) {
    String mac = line.substring(14); mac.trim();
    if (mac.length() < 17) { Serial.println("[ERR] connect mouse xx:xx:xx:xx:xx:xx"); return; }
    if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt=0; isManualScan=false; }
    mouseReconnectAt=0; mouseReconnectFails=0;
    { NimBLEAddress a(mac.c_str(), 1); NimBLEDevice::deleteBond(a); } delay(100);
    NimBLEAddress addr(mac.c_str(), 1); bool ok=false; int attempt=0;
    Serial.println("[MOUSE] Connecting — keep mouse in pairing mode...");
    while (!ok) {
      attempt++;
      Serial.printf("[MOUSE] Attempt %d\n", attempt);
      ok = tryConnectMouse(addr);
      if (!ok) {
        if (attempt >= 20) { Serial.println("[MOUSE] Giving up. Try again."); mouseReconnectAt=millis()+10000; break; }
        if (Serial.available()) { while(Serial.available()) Serial.read(); _serialBuf=""; Serial.println("[MOUSE] Aborted."); break; }
        delay(500);
      }
    }
    if (ok) { saveMouse(pClientMouse->getPeerAddress()); saveMouseSettings(); }

  } else if (line == "forget kb") {
    if (pClientKb && pClientKb->isConnected()) pClientKb->disconnect();
    prefs.begin(NVS_NS,false); prefs.remove("kb-mac"); prefs.remove("kb-type"); prefs.end();
    memset(kbMAC,0,sizeof(kbMAC)); kbReconnectAt=0;
    memset(prevKeys,0,6); prevMod=0;
    Serial.println("[NVS] Keyboard forgotten.");

  } else if (line == "forget mouse") {
    if (pClientMouse && pClientMouse->isConnected()) pClientMouse->disconnect();
    prefs.begin(NVS_NS,false);
    prefs.remove("ms-mac"); prefs.remove("ms-type"); prefs.remove("ms-proto");
    prefs.end();
    memset(mouseMAC,0,sizeof(mouseMAC)); mouseReconnectAt=0;
    g_ps2ProtoMode = 0x04; // reset to default
    portENTER_CRITICAL(&g_mux); g_accX=g_accY=g_accW=0; g_buttons=0; g_dirty=false; portEXIT_CRITICAL(&g_mux);
    g_prevButtons=0;
    Serial.println("[NVS] Mouse forgotten. Proto reset to Explorer (0x04).");

  } else if (line == "forget all") {
    if (pClientKb    && pClientKb->isConnected())    pClientKb->disconnect();
    if (pClientMouse && pClientMouse->isConnected()) pClientMouse->disconnect();
    prefs.begin(NVS_NS,false);
    prefs.remove("kb-mac"); prefs.remove("kb-type");
    prefs.remove("ms-mac"); prefs.remove("ms-type");
    prefs.remove("ms-scale"); prefs.remove("ms-flipy"); prefs.remove("ms-flipw");
    prefs.remove("ms-rid");   prefs.remove("ms-proto");
    prefs.end();
    NimBLEDevice::deleteAllBonds();
    memset(kbMAC,0,sizeof(kbMAC)); memset(mouseMAC,0,sizeof(mouseMAC));
    kbReconnectAt=mouseReconnectAt=0;
    g_scaleDivisor=4; g_flipY=false; g_flipW=false; g_filterReportId=0; g_ps2ProtoMode=0x04;
    memset(prevKeys,0,6); prevMod=0;
    portENTER_CRITICAL(&g_mux); g_accX=g_accY=g_accW=0; g_buttons=0; g_dirty=false; portEXIT_CRITICAL(&g_mux);
    g_prevButtons=0;
    Serial.println("[NVS] All forgotten — settings reset. Proto reset to Explorer (0x04).");

  } else if (line.startsWith("scale ")) {
    int n = line.substring(6).toInt();
    if (n>=1 && n<=64) { g_scaleDivisor=n; saveMouseSettings(); Serial.printf("[CFG] Scale 1/%d saved.\n",n); }
    else Serial.println("[CFG] scale 1-64");

  } else if (line == "flipy") {
    g_flipY=!g_flipY; saveMouseSettings();
    Serial.printf("[CFG] FlipY: %s saved.\n", g_flipY?"ON":"OFF");

  } else if (line == "flipw") {
    g_flipW=!g_flipW; saveMouseSettings();
    Serial.printf("[CFG] FlipW: %s saved.\n", g_flipW?"ON":"OFF");

  } else if (line.startsWith("reportid ")) {
    int n = line.substring(9).toInt();
    if (n>=0 && n<=255) { g_filterReportId=(uint8_t)n; saveMouseSettings(); Serial.printf("[CFG] ReportID %d saved. Reconnect mouse to apply.\n",n); }
    else Serial.println("[CFG] reportid 0-255");

  } else if (line.startsWith("proto ")) {
    int n = line.substring(6).toInt();
    if (n == 0 || n == 3 || n == 4) {
      g_ps2ProtoMode = (uint8_t)n;
      saveMouseSettings();
      Serial.printf("[CFG] PS/2 proto: %s saved.\n", ps2ProtoName(g_ps2ProtoMode));
      Serial.println("[CFG] Takes effect on next PS/2 Reset from host (driver reload).");
    } else {
      Serial.println("[CFG] proto accepts: 0  3  4");
      Serial.println("        0 = Standard (3-btn, no scroll)");
      Serial.println("        3 = IntelliMouse (scroll wheel)");
      Serial.println("        4 = Explorer (scroll+Back+Fwd)  [default]");
    }

  } else if (line == "status") {
    Serial.print(buildStatus());

  } else {
    Serial.printf("[CMD] Unknown: '%s'\n", line.c_str());
  }
}

// ════════════════════════════════════════════════════════════════════════════════
//  BLE DAEMON TASK — monitors both connections independently of loop()
// ════════════════════════════════════════════════════════════════════════════════

void bleDaemonTask(void* arg) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ── Battery keepalives — readValue() is blocking (200-600 ms BLE round-trip).
    // Running here on Core 0 FreeRTOS task keeps loop() on Core 1 non-blocking,
    // so PS/2 Reset handling and mouse movement are never delayed.
    // forceRead=true is required — NimBLE caches the last value by default,
    // which means readValue() without true always returns the value from connect time.
    if (kbKeepaliveAt && millis() >= kbKeepaliveAt && pKbBatChar && pClientKb && pClientKb->isConnected()) {
      kbKeepaliveAt = millis() + KEEPALIVE_MS;
      // Read to keep keyboard awake — result discarded.
      // Battery value is maintained by the notification callback which provides
      // the correct current value. readValue() returns a stale cached value.
      pKbBatChar->readValue();
    }
    if (mouseKeepaliveAt && millis() >= mouseKeepaliveAt && pMouseBatChar && pClientMouse && pClientMouse->isConnected()) {
      mouseKeepaliveAt = millis() + KEEPALIVE_MS;
      pMouseBatChar->readValue(); // keepalive only — result discarded
    }

    // Keyboard
    if (!isManualScan && strlen(kbMAC) && !(pClientKb && pClientKb->isConnected())
        && !kbReconnectAt && !kbConnecting) {
      logQ("[DAEMON] Keyboard lost — scheduling reconnect...\n");
      memset(prevKeys,0,6); prevMod=0; typematicKey=0; typematicArmed=false;
      if (!kbReconnectScan && !mouseReconnectScan) {
        kbReconnectScan=true;
        NimBLEDevice::getScan()->start(3000, false);
        logQ("[SCAN] Waiting for keyboard...\n");
      } else {
        kbReconnectAt = millis() + 2000;
      }
    }

    // Mouse
    if (!isManualScan && strlen(mouseMAC) && !(pClientMouse && pClientMouse->isConnected())
        && !mouseReconnectAt && !mouseConnecting) {
      logQ("[DAEMON] Mouse lost — scheduling reconnect...\n");
      portENTER_CRITICAL(&g_mux); g_accX=g_accY=g_accW=0; g_buttons=0; g_dirty=false; portEXIT_CRITICAL(&g_mux);
      g_prevButtons=0;
      if (!kbReconnectScan && !mouseReconnectScan) {
        mouseReconnectScan=true;
        NimBLEDevice::getScan()->start(3000, false);
        logQ("[SCAN] Waiting for mouse...\n");
      } else {
        mouseReconnectAt = millis() + 2000;
      }
    }
  }
}

// ════════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════════

void setup() {
  // PS/2 devices first — BIOS expects BAT 0xAA within ~500 ms of power-on.
  // Serial.begin + delay(200) + WiFi deinit together take ~400 ms which can
  // push BAT past the BIOS timeout on slow boots. begin() contains its own
  // 200 ms stabilisation delay, so BAT arrives ~200 ms after reset.
  keyboard.begin();
  delay(50);
  mouse.begin();

  Serial.begin(115200);
  delay(200);
  // Serial.setTimeout not needed — handleSerial() uses a non-blocking
  // character accumulator that never calls readStringUntil().

  Serial.println("[PS2KB] BAT sent");    // begin() ran before Serial.begin()
  Serial.println("[PS2MOUSE] BAT+ID sent");

  esp_wifi_stop();
  esp_wifi_deinit();

  // BLE — must init after PS/2 tasks (NimBLE uses Core 0, same as ps2 tasks; scheduler handles it)
  NimBLEDevice::init(BRIDGE_NAME);
  NimBLEDevice::setPower(9);
  NimBLEDevice::setSecurityAuth(true, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::getScan()->setScanCallbacks(new MyScanCallbacks(), false);
  NimBLEDevice::getScan()->setActiveScan(true);
  NimBLEDevice::getScan()->setInterval(50);
  NimBLEDevice::getScan()->setWindow(45);

  // Daemon task — monitors both connections, triggers reconnect on loss
  xTaskCreatePinnedToCore(bleDaemonTask, "ble_daemon", 8192, nullptr, 1, nullptr, 0);

  // Load saved devices and mouse settings from NVS
  loadSavedDevices();
  loadMouseSettings();

  Serial.println("[NVS] ========== Stored configuration ==========");
  Serial.printf( "[NVS] Keyboard MAC:  %s\n",  strlen(kbMAC)    ? kbMAC    : "(none)");
  Serial.printf( "[NVS] Mouse MAC:     %s\n",  strlen(mouseMAC) ? mouseMAC : "(none)");
  Serial.println("[NVS] --- Mouse settings ---");
  Serial.printf( "[NVS] Scale:         1/%d\n", (int)g_scaleDivisor);
  Serial.printf( "[NVS] FlipY:         %s\n",  g_flipY ? "on" : "off");
  Serial.printf( "[NVS] FlipW:         %s\n",  g_flipW ? "on" : "off");
  Serial.printf( "[NVS] ReportID:      %d%s\n", (int)g_filterReportId, g_filterReportId ? "" : " (auto)");
  Serial.printf( "[NVS] PS/2 Proto:    %s\n",  ps2ProtoName(g_ps2ProtoMode));
  Serial.println("[NVS] ============================================");

  if (strlen(kbMAC))    kbReconnectAt    = millis() + 500;
  if (strlen(mouseMAC)) mouseReconnectAt = millis() + 800;

  printHelp();
}

// ════════════════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════════════════

void loop() {
  logDrain();      // print any messages queued by Core 0 tasks
  handleSerial();

  // ── Scan end ─────────────────────────────────────────────────────────────────
  if (scanEndAt && millis() >= scanEndAt) {
    scanEndAt = 0; isManualScan = false;
    NimBLEDevice::getScan()->stop();
    Serial.printf("[SCAN] Done — %d device(s). Use: connect kb/mouse <mac>\n", scanCount);
  }

  // ── Keyboard disconnect ───────────────────────────────────────────────────────
  if (pClientKb && !pClientKb->isConnected() && strlen(kbMAC) && !kbReconnectAt) {
    Serial.println("[KB] Disconnected.");
    NimBLEDevice::deleteClient(pClientKb); pClientKb=nullptr;
    memset(prevKeys,0,6); prevMod=0;
    kbReconnectFails=0; kbReconnectScan=true;
    NimBLEDevice::getScan()->start(3000, false);
  }

  // ── Mouse disconnect ──────────────────────────────────────────────────────────
  if (pClientMouse && !pClientMouse->isConnected() && strlen(mouseMAC) && !mouseReconnectAt) {
    Serial.println("[MOUSE] Disconnected.");
    NimBLEDevice::deleteClient(pClientMouse); pClientMouse=nullptr;
    portENTER_CRITICAL(&g_mux); g_accX=g_accY=g_accW=0; g_buttons=0; g_dirty=false; portEXIT_CRITICAL(&g_mux);
    g_prevButtons=0;
    mouseReconnectFails=0; mouseReconnectScan=true;
    NimBLEDevice::getScan()->start(3000, false);
  }

  // ── Keyboard reconnect ────────────────────────────────────────────────────────
  if (kbReconnectAt && millis() >= kbReconnectAt && strlen(kbMAC) && !kbConnecting) {
    kbReconnectAt = 0;
    if (!pClientKb || !pClientKb->isConnected()) {
      if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt=0; }
      kbConnecting = true;
      if (tryConnectKb(NimBLEAddress(kbMAC, kbType))) {
        kbReconnectFails=0;
      } else {
        ++kbReconnectFails;
        Serial.printf("[KB] Connect failed (%dx)\n", kbReconnectFails);
        kbReconnectScan=true;
        // Exponential-ish backoff: wait longer after repeated failures (max 10s)
        unsigned long backoff = min(2000UL + (unsigned long)kbReconnectFails * 500UL, 10000UL);
        kbReconnectAt = millis() + backoff;
      }
      kbConnecting = false;
    }
  }

  // ── Mouse reconnect ───────────────────────────────────────────────────────────
  if (mouseReconnectAt && millis() >= mouseReconnectAt && strlen(mouseMAC) && !mouseConnecting) {
    mouseReconnectAt = 0;
    if (!pClientMouse || !pClientMouse->isConnected()) {
      if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt=0; }
      mouseConnecting = true;
      if (tryConnectMouse(NimBLEAddress(mouseMAC, mouseType))) {
        mouseReconnectFails=0;
      } else {
        ++mouseReconnectFails;
        Serial.printf("[MOUSE] Connect failed (%dx)\n", mouseReconnectFails);
        mouseReconnectScan=true;
        unsigned long backoff = min(2000UL + (unsigned long)mouseReconnectFails * 500UL, 10000UL);
        mouseReconnectAt = millis() + backoff;
      }
      mouseConnecting = false;
    }
  }

  // ── Forward LED state to BLE keyboard ────────────────────────────────────────
  if (pendingLedMask != 0xFF && pKbLedChar) {
    uint8_t led = pendingLedMask; pendingLedMask = 0xFF;
    pKbLedChar->writeValue(&led, 1, false);
    Serial.printf("[LED] BLE LED mask 0x%02X\n", led);
  }

  // ── Hotkeys — type battery or status via PS/2 keyboard ───────────────────────
  if (batteryRequested) {
    batteryRequested = false;
    typematicKey=0; typematicNext=0;
    keyboard.keyHid_send(0xE0, false); // LCtrl release
    keyboard.keyHid_send(0xE2, false); // LAlt  release
    keyboard.keyHid_send(0xE1, false); // LShift release
    delay(50);
    typeBatteryViaPS2();
  }
  if (statusRequested) {
    statusRequested = false;
    typematicKey=0; typematicNext=0;
    keyboard.keyHid_send(0xE0, false);
    keyboard.keyHid_send(0xE2, false);
    keyboard.keyHid_send(0xE1, false);
    delay(50);
    typeStatusViaPS2();
  }

  // ── Typematic key repeat ──────────────────────────────────────────────────────
  if (typematicKey && typematicNext && millis() >= typematicNext) {
    if (!typematicArmed) { typematicArmed=true; typematicNext=millis()+TYPEMATIC_RATE_MS; }
    else                 { typematicNext=millis()+TYPEMATIC_RATE_MS; }
    keyboard.keyHid_send(typematicKey, true);
  }

  // ── Mouse movement → PS/2 ────────────────────────────────────────────────────
  processMouseMovement();

  delay(1);
}
