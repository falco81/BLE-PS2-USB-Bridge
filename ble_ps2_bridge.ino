#include <NimBLEDevice.h>
#include <Preferences.h>
#include "esp_wifi.h"

// LED state pending forward to BLE keyboard — set by PS/2 class, read in loop()
// 0xFF = no pending update
static volatile uint8_t pendingLedMask = 0xFF;

// ── PS/2 keyboard driver (from ps2kb.h / ps2kb.cpp) ─────────────────────────
// ps2kb.h — Arduino port of esp32-ps2dev (keyboard only)
// Based on: https://github.com/hamberthm/esp32-bt2ps2
// Ported for Arduino/NimBLE use, mouse removed, NVS removed.


// ── Timing constants (from reference project) ────────────────────────────────
#define PS2KB_CLK_HALF_US        40    // half clock period µs (spec: 30-50)
#define PS2KB_CLK_QUARTER_US     20
#define PS2KB_BYTE_INTERVAL_US  500    // gap between bytes
#define PS2KB_HOST_POLL_MS        9    // how often to check for host commands
#define PS2KB_PACKET_QUEUE_LEN   20

// ── Scan code set 2 — make/break tables ──────────────────────────────────────
// Source: http://www.computer-engineering.org/ps2keyboard/scancodes2.html

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

// ── Packet ────────────────────────────────────────────────────────────────────
struct PS2Packet {
  uint8_t len;
  uint8_t data[16];
};

// ── PS2Keyboard class ─────────────────────────────────────────────────────────
class PS2Keyboard {
public:
  PS2Keyboard(int clk, int data);

  // Call once in setup() — starts FreeRTOS tasks, sends BAT 0xAA
  void begin();

  // Send HID keycode as PS/2 scan code (used from BLE notify callback)
  void keyHid_send(uint8_t hidkey, bool keyDown);

  // Internal — used by FreeRTOS tasks
  enum class BusState { IDLE, INHIBITED, HOST_REQUEST_TO_SEND };
  BusState get_bus_state();
  int  ps2_write(uint8_t data);
  int  ps2_write_wait_idle(uint8_t data, uint64_t timeout_us = 1500);
  int  ps2_read(uint8_t *data, uint64_t timeout_ms = 0);
  void reply_to_host(uint8_t cmd);
  int  send_packet(PS2Packet *pkt);
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

// ── FreeRTOS task functions ───────────────────────────────────────────────────
void ps2kb_task_host(void *arg);
void ps2kb_task_send(void *arg);

// ps2kb.cpp — Arduino port of esp32-ps2dev (keyboard only)
// Logic copied 1:1 from reference project, Arduino wrappers removed
// (Arduino already provides digitalWrite/digitalRead/millis/micros/delay)


// ── Scan code tables (set 2) ──────────────────────────────────────────────────
// Only MAKE (press) and BREAK (release) — indexed by PS2Key enum

static const uint8_t MAKE[][9] = {
  {1,0x1C},{1,0x32},{1,0x21},{1,0x23},{1,0x24},{1,0x2B},{1,0x34},{1,0x33}, // A-H
  {1,0x43},{1,0x3B},{1,0x42},{1,0x4B},{1,0x3A},{1,0x31},{1,0x44},{1,0x4D}, // I-P
  {1,0x15},{1,0x2D},{1,0x1B},{1,0x2C},{1,0x3C},{1,0x2A},{1,0x1D},{1,0x22}, // Q-X
  {1,0x35},{1,0x1A},                                                          // Y-Z
  {1,0x45},{1,0x16},{1,0x1E},{1,0x26},{1,0x25},{1,0x2E},{1,0x36},{1,0x3D}, // 0-7
  {1,0x3E},{1,0x46},                                                          // 8-9
  {1,0x0E},{1,0x4E},{1,0x55},{1,0x5D},{1,0x66},{1,0x29},                   // ` - . BKSP SPC
  {1,0x0D},{1,0x58},                                                          // TAB CAPS
  {1,0x12},{1,0x14},{2,0xE0,0x1F},{1,0x11},                                 // LSHIFT LCTRL LSUPER LALT
  {1,0x59},{2,0xE0,0x14},{2,0xE0,0x27},{2,0xE0,0x11},{2,0xE0,0x2F},       // RSHIFT RCTRL RSUPER RALT MENU
  {1,0x5A},{1,0x76},                                                          // ENTER ESC
  {1,0x05},{1,0x06},{1,0x04},{1,0x0C},{1,0x03},{1,0x0B},                   // F1-F6
  {1,0x83},{1,0x0A},{1,0x01},{1,0x09},{1,0x78},{1,0x07},                   // F7-F12
  {4,0xE0,0x12,0xE0,0x7C},{1,0x7E},{8,0xE1,0x14,0x77,0xE1,0xF0,0x14,0xF0,0x77}, // PRINT SCRL PAUSE
  {1,0x54},                                                                   // [
  {2,0xE0,0x70},{2,0xE0,0x6C},{2,0xE0,0x7D},                               // INS HOME PGUP
  {2,0xE0,0x71},{2,0xE0,0x69},{2,0xE0,0x7A},                               // DEL END PGDN
  {2,0xE0,0x75},{2,0xE0,0x6B},{2,0xE0,0x72},{2,0xE0,0x74},                // UP LEFT DOWN RIGHT
  {1,0x77},                                                                   // NUMLOCK
  {2,0xE0,0x4A},{1,0x7C},{1,0x7B},{1,0x79},{2,0xE0,0x5A},{1,0x71},       // KP/ KP* KP- KP+ KPENT KP.
  {1,0x70},{1,0x69},{1,0x72},{1,0x7A},{1,0x6B},{1,0x73},{1,0x74},         // KP0-KP6
  {1,0x6C},{1,0x75},{1,0x7D},                                               // KP7-KP9
  {1,0x5B},{1,0x4C},{1,0x52},{1,0x41},{1,0x49},{1,0x4A},                   // ] ; ' , . /
  {2,0xE0,0x37},{2,0xE0,0x3F},{2,0xE0,0x5E},                               // ACPI PWR SLP WKE
  {2,0xE0,0x4D},{2,0xE0,0x15},{2,0xE0,0x3B},{2,0xE0,0x34},               // MEDIA NEXT PREV STOP PLAY
  {2,0xE0,0x23},{2,0xE0,0x32},{2,0xE0,0x21},                               // MUTE VOL+ VOL-
  {2,0xE0,0x50},{2,0xE0,0x48},{2,0xE0,0x2B},{2,0xE0,0x40},               // MSEL EMAIL CALC MYPC
  {2,0xE0,0x10},{2,0xE0,0x3A},{2,0xE0,0x38},{2,0xE0,0x30},               // WSRCH WHOME WBACK WFWD
  {2,0xE0,0x28},{2,0xE0,0x20},{2,0xE0,0x18},                               // WSTOP WREF WFAV
};

// BREAK = prepend F0 to each byte, E0 prefix stays before F0
// We build break codes dynamically in _keyup() to save RAM

// ── Helpers ───────────────────────────────────────────────────────────────────

static uint64_t ps2_micros() { return esp_timer_get_time(); }

static void ps2_delay_us(uint32_t us) {
  uint64_t end = ps2_micros() + us;
  while (ps2_micros() < end) { asm volatile("nop"); }
}

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

// ── Low-level write (device→host) ─────────────────────────────────────────────
int PS2Keyboard::ps2_write(uint8_t data) {
  if (get_bus_state() != BusState::IDLE) return -1;

  uint8_t parity = 1;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  taskENTER_CRITICAL(&mux);

  _golo(_dat);
  ps2_delay_us(PS2KB_CLK_QUARTER_US);
  _golo(_clk); // start bit
  ps2_delay_us(PS2KB_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2KB_CLK_QUARTER_US);

  for (int i = 0; i < 8; i++) {
    if (data & 0x01) _gohi(_dat); else _golo(_dat);
    ps2_delay_us(PS2KB_CLK_QUARTER_US);
    _golo(_clk);
    ps2_delay_us(PS2KB_CLK_HALF_US);
    _gohi(_clk);
    ps2_delay_us(PS2KB_CLK_QUARTER_US);
    parity ^= (data & 0x01);
    data >>= 1;
  }

  // parity
  if (parity) _gohi(_dat); else _golo(_dat);
  ps2_delay_us(PS2KB_CLK_QUARTER_US);
  _golo(_clk);
  ps2_delay_us(PS2KB_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2KB_CLK_QUARTER_US);

  // stop bit
  _gohi(_dat);
  ps2_delay_us(PS2KB_CLK_QUARTER_US);
  _golo(_clk);
  ps2_delay_us(PS2KB_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2KB_CLK_QUARTER_US);

  taskEXIT_CRITICAL(&mux);
  return 0;
}

int PS2Keyboard::ps2_write_wait_idle(uint8_t data, uint64_t timeout_us) {
  uint64_t start = ps2_micros();
  while (get_bus_state() != BusState::IDLE) {
    if (ps2_micros() - start > timeout_us) return -1;
  }
  return ps2_write(data);
}

// ── Low-level read (host→device) ─────────────────────────────────────────────
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

  ps2_delay_us(PS2KB_CLK_QUARTER_US);
  _golo(_clk);
  ps2_delay_us(PS2KB_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2KB_CLK_QUARTER_US);

  for (int i = 0; i < 8; i++) {
    if (gpio_get_level((gpio_num_t)_dat)) { data |= bit; calc_parity ^= 1; }
    bit <<= 1;
    ps2_delay_us(PS2KB_CLK_QUARTER_US);
    _golo(_clk);
    ps2_delay_us(PS2KB_CLK_HALF_US);
    _gohi(_clk);
    ps2_delay_us(PS2KB_CLK_QUARTER_US);
  }

  if (gpio_get_level((gpio_num_t)_dat)) recv_parity = 1;

  // stop bit
  ps2_delay_us(PS2KB_CLK_QUARTER_US);
  _golo(_clk);
  ps2_delay_us(PS2KB_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2KB_CLK_QUARTER_US);

  // ACK
  ps2_delay_us(PS2KB_CLK_QUARTER_US);
  _golo(_dat);
  _golo(_clk);
  ps2_delay_us(PS2KB_CLK_HALF_US);
  _gohi(_clk);
  ps2_delay_us(PS2KB_CLK_QUARTER_US);
  _gohi(_dat);

  taskEXIT_CRITICAL(&mux);

  *value = data;
  return (recv_parity == calc_parity) ? 0 : -2;
}

void PS2Keyboard::_ack() {
  ps2_delay_us(PS2KB_BYTE_INTERVAL_US);
  ps2_write(0xFA);
  ps2_delay_us(PS2KB_BYTE_INTERVAL_US);
}

// ── Host command handler ──────────────────────────────────────────────────────
void PS2Keyboard::reply_to_host(uint8_t cmd) {
  uint8_t val;
  Serial.printf("[PS/2] cmd 0x%02X\n", cmd);

  switch (cmd) {
    case 0xFF: // RESET
      _data_reporting = false;
      _ack();
      while (ps2_write(0xAA) != 0) vTaskDelay(1);
      _data_reporting = true;
      break;

    case 0xFE: // RESEND
      _ack();
      break;

    case 0xF6: // SET DEFAULTS
      _ack();
      break;

    case 0xF5: // DISABLE DATA REPORTING
      _data_reporting = false;
      _ack();
      break;

    case 0xF4: // ENABLE DATA REPORTING
      _data_reporting = true;
      _ack();
      break;

    case 0xF3: // SET TYPEMATIC RATE
      _ack();
      if (ps2_read(&val) == 0) _ack();
      break;

    case 0xF2: // GET DEVICE ID — critical! BIOS decides kb type on this
      _ack();
      while (ps2_write(0xAB) != 0) vTaskDelay(1);
      while (ps2_write(0x83) != 0) vTaskDelay(1);
      break;

    case 0xF0: // SET SCAN CODE SET
      _ack();
      if (ps2_read(&val) == 0) _ack();
      break;

    case 0xEE: // ECHO
      ps2_delay_us(PS2KB_BYTE_INTERVAL_US);
      ps2_write(0xEE);
      ps2_delay_us(PS2KB_BYTE_INTERVAL_US);
      break;

    case 0xED: // SET/RESET LEDs
      ps2_delay_us(PS2KB_BYTE_INTERVAL_US);
      while (ps2_write(0xFA) != 0) vTaskDelay(1);
      ps2_delay_us(PS2KB_BYTE_INTERVAL_US);
      if (ps2_read(&val, 10) == 0) {
        ps2_delay_us(PS2KB_BYTE_INTERVAL_US);
        while (ps2_write(0xFA) != 0) vTaskDelay(1);
        ps2_delay_us(PS2KB_BYTE_INTERVAL_US);
        // PS/2 LED bits: 0=ScrollLock 1=NumLock 2=CapsLock
        // BLE HID LED bits: 0=NumLock  1=CapsLock  2=ScrollLock
        Serial.printf("[PS/2] LED mask 0x%02X\n", val);
        // Remap and post — forwarded to BLE in loop() to avoid class scope issue
        uint8_t bleLed = 0;
        if (val & 0x02) bleLed |= 0x01; // NumLock
        if (val & 0x04) bleLed |= 0x02; // CapsLock
        if (val & 0x01) bleLed |= 0x04; // ScrollLock
        pendingLedMask = bleLed;
      }
      break;

    default:
      _ack();
      break;
  }
}

// ── begin() ──────────────────────────────────────────────────────────────────
void PS2Keyboard::begin() {
  // Configure pins open-drain (same as reference project app_main)
  gpio_config_t io = {};
  io.intr_type    = GPIO_INTR_DISABLE;
  io.mode         = GPIO_MODE_OUTPUT_OD;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.pull_up_en   = GPIO_PULLUP_DISABLE;

  io.pin_bit_mask = (1ULL << _clk);
  gpio_config(&io);
  io.pin_bit_mask = (1ULL << _dat);
  gpio_config(&io);

  _gohi(_clk);
  _gohi(_dat);

  _mutex = xSemaphoreCreateMutex();
  _queue = xQueueCreate(PS2KB_PACKET_QUEUE_LEN, sizeof(PS2Packet));

  // Send BAT before starting tasks — BIOS expects it within ~500 ms of power-on
  xSemaphoreTake(_mutex, portMAX_DELAY);
  ps2_delay_us(PS2KB_BYTE_INTERVAL_US);
  vTaskDelay(pdMS_TO_TICKS(200));
  ps2_write(0xAA);
  xSemaphoreGive(_mutex);

  // Core 0 — same as reference project (BLE/Bluetooth also on core 0; that's fine,
  // scheduler handles it; reference project runs kb tasks on APP_CPU = core 0 by default)
  xTaskCreatePinnedToCore(ps2kb_task_host, "ps2host", 4096, this, 10, &_task_host, 0);
  xTaskCreatePinnedToCore(ps2kb_task_send, "ps2send", 4096, this,  9, &_task_send, 0);

  Serial.println("[PS/2] begin() done — BAT sent, tasks started");
}

// ── send_packet() ─────────────────────────────────────────────────────────────
int PS2Keyboard::send_packet(PS2Packet *pkt) {
  return (xQueueSend(_queue, pkt, 0) == pdTRUE) ? 0 : -1;
}

// ── _keydown / _keyup ─────────────────────────────────────────────────────────
void PS2Keyboard::_keydown(PS2Key key) {
  if (!_data_reporting) return;
  PS2Packet pkt;
  // MAKE table: first byte = length, then scan codes
  pkt.len = MAKE[key][0];
  for (int i = 0; i < pkt.len; i++) pkt.data[i] = MAKE[key][i + 1];
  send_packet(&pkt);
}

void PS2Keyboard::_keyup(PS2Key key) {
  if (!_data_reporting) return;
  PS2Packet pkt;
  pkt.len = 0;
  // Build break code: for multi-byte make codes, insert F0 before last byte,
  // keep E0/E1 prefixes in front.
  // Algorithm: copy all prefix bytes (E0/E1), then append F0, then last byte.
  uint8_t makelen = MAKE[key][0];
  const uint8_t *make = &MAKE[key][1];

  if (key == K_PAUSE) {
    // Pause has no break code
    return;
  }

  // Copy E0/E1 prefix bytes
  int i = 0;
  while (i < makelen - 1 && (make[i] == 0xE0 || make[i] == 0xE1)) {
    pkt.data[pkt.len++] = make[i++];
  }
  pkt.data[pkt.len++] = 0xF0;
  // remaining bytes (last scancode byte)
  while (i < makelen) {
    pkt.data[pkt.len++] = make[i++];
  }
  send_packet(&pkt);
}

// ── keyHid_send — BLE HID keycode → PS/2 ─────────────────────────────────────
void PS2Keyboard::keyHid_send(uint8_t hid, bool down) {
  PS2Key key;
  switch (hid) {
    case 0x04: key=K_A; break;      case 0x05: key=K_B; break;
    case 0x06: key=K_C; break;      case 0x07: key=K_D; break;
    case 0x08: key=K_E; break;      case 0x09: key=K_F; break;
    case 0x0A: key=K_G; break;      case 0x0B: key=K_H; break;
    case 0x0C: key=K_I; break;      case 0x0D: key=K_J; break;
    case 0x0E: key=K_K; break;      case 0x0F: key=K_L; break;
    case 0x10: key=K_M; break;      case 0x11: key=K_N; break;
    case 0x12: key=K_O; break;      case 0x13: key=K_P; break;
    case 0x14: key=K_Q; break;      case 0x15: key=K_R; break;
    case 0x16: key=K_S; break;      case 0x17: key=K_T; break;
    case 0x18: key=K_U; break;      case 0x19: key=K_V; break;
    case 0x1A: key=K_W; break;      case 0x1B: key=K_X; break;
    case 0x1C: key=K_Y; break;      case 0x1D: key=K_Z; break;
    case 0x1E: key=K_1; break;      case 0x1F: key=K_2; break;
    case 0x20: key=K_3; break;      case 0x21: key=K_4; break;
    case 0x22: key=K_5; break;      case 0x23: key=K_6; break;
    case 0x24: key=K_7; break;      case 0x25: key=K_8; break;
    case 0x26: key=K_9; break;      case 0x27: key=K_0; break;
    case 0x28: key=K_RETURN; break; case 0x29: key=K_ESCAPE; break;
    case 0x2A: key=K_BACKSPACE; break; case 0x2B: key=K_TAB; break;
    case 0x2C: key=K_SPACE; break;  case 0x2D: key=K_MINUS; break;
    case 0x2E: key=K_EQUALS; break; case 0x2F: key=K_LEFTBRACKET; break;
    case 0x30: key=K_RIGHTBRACKET; break; case 0x31: key=K_BACKSLASH; break;
    case 0x33: key=K_SEMICOLON; break;    case 0x34: key=K_QUOTE; break;
    case 0x35: key=K_BACKQUOTE; break;    case 0x36: key=K_COMMA; break;
    case 0x37: key=K_PERIOD; break;       case 0x38: key=K_SLASH; break;
    case 0x39: key=K_CAPSLOCK; break;
    case 0x3A: key=K_F1; break;  case 0x3B: key=K_F2; break;
    case 0x3C: key=K_F3; break;  case 0x3D: key=K_F4; break;
    case 0x3E: key=K_F5; break;  case 0x3F: key=K_F6; break;
    case 0x40: key=K_F7; break;  case 0x41: key=K_F8; break;
    case 0x42: key=K_F9; break;  case 0x43: key=K_F10; break;
    case 0x44: key=K_F11; break; case 0x45: key=K_F12; break;
    case 0x46: key=K_PRINT; break;    case 0x47: key=K_SCROLLOCK; break;
    case 0x48: key=K_PAUSE; break;    case 0x49: key=K_INSERT; break;
    case 0x4A: key=K_HOME; break;     case 0x4B: key=K_PAGEUP; break;
    case 0x4C: key=K_DELETE; break;   case 0x4D: key=K_END; break;
    case 0x4E: key=K_PAGEDOWN; break;
    case 0x4F: key=K_RIGHT; break;    case 0x50: key=K_LEFT; break;
    case 0x51: key=K_DOWN; break;     case 0x52: key=K_UP; break;
    case 0x53: key=K_NUMLOCK; break;
    case 0x54: key=K_KP_DIVIDE; break;   case 0x55: key=K_KP_MULTIPLY; break;
    case 0x56: key=K_KP_MINUS; break;    case 0x57: key=K_KP_PLUS; break;
    case 0x58: key=K_KP_ENTER; break;
    case 0x59: key=K_KP1; break; case 0x5A: key=K_KP2; break;
    case 0x5B: key=K_KP3; break; case 0x5C: key=K_KP4; break;
    case 0x5D: key=K_KP5; break; case 0x5E: key=K_KP6; break;
    case 0x5F: key=K_KP7; break; case 0x60: key=K_KP8; break;
    case 0x61: key=K_KP9; break; case 0x62: key=K_KP0; break;
    case 0x63: key=K_KP_PERIOD; break; case 0x65: key=K_MENU; break;
    case 0x66: key=K_ACPI_POWER; break;
    case 0x74: key=K_MEDIA_PLAY_PAUSE; break;
    case 0x78: key=K_MEDIA_STOP; break;
    case 0x7F: key=K_MEDIA_MUTE; break;
    case 0x80: key=K_MEDIA_VOLUME_UP; break;
    case 0x81: key=K_MEDIA_VOLUME_DOWN; break;
    case 0xE0: key=K_LCTRL; break;  case 0xE1: key=K_LSHIFT; break;
    case 0xE2: key=K_LALT; break;   case 0xE3: key=K_LSUPER; break;
    case 0xE4: key=K_RCTRL; break;  case 0xE5: key=K_RSHIFT; break;
    case 0xE6: key=K_RALT; break;   case 0xE7: key=K_RSUPER; break;
    default: return;
  }
  if (down) _keydown(key); else _keyup(key);
}

// ── FreeRTOS tasks ────────────────────────────────────────────────────────────

void ps2kb_task_host(void *arg) {
  PS2Keyboard *kb = (PS2Keyboard *)arg;
  while (true) {
    xSemaphoreTake(kb->get_mutex(), portMAX_DELAY);
    if (kb->get_bus_state() == PS2Keyboard::BusState::HOST_REQUEST_TO_SEND) {
      uint8_t cmd;
      if (kb->ps2_read(&cmd) == 0) {
        kb->reply_to_host(cmd);
      }
    }
    xSemaphoreGive(kb->get_mutex());
    vTaskDelay(pdMS_TO_TICKS(PS2KB_HOST_POLL_MS));
  }
}

void ps2kb_task_send(void *arg) {
  PS2Keyboard *kb = (PS2Keyboard *)arg;
  while (true) {
    PS2Packet pkt;
    if (xQueueReceive(kb->get_queue(), &pkt, portMAX_DELAY) == pdTRUE) {
      xSemaphoreTake(kb->get_mutex(), portMAX_DELAY);
      ps2_delay_us(PS2KB_BYTE_INTERVAL_US);
      for (int i = 0; i < pkt.len; i++) {
        kb->ps2_write_wait_idle(pkt.data[i]);
        ps2_delay_us(PS2KB_BYTE_INTERVAL_US);
      }
      xSemaphoreGive(kb->get_mutex());
    }
  }
}

// ── Main sketch ─────────────────────────────────────────────────────────────
/*
 * BLE HID Host → PS/2 AT Keyboard Bridge  v1.6
 * ESP32-WROOM-32 | NimBLE-Arduino 2.x | Arduino IDE 2.x
 *
 * Wiring (with 3.3V↔5V level shifter):
 *   PS/2 CLK  ←→ level shifter ←→ GPIO19
 *   PS/2 DATA ←→ level shifter ←→ GPIO18
 *   PS/2 GND  ————————————————————— GND
 *   PS/2 +5V  ————————————————————— VIN
 *
 * Serial commands (115200 baud):
 *   scan              Scan BLE 10 seconds
 *   connect <mac>     Connect (3 attempts) and save
 *   forget            Clear saved keyboard and bonds
 *   status            Show connection status
 *   help              Show help
 *
 * Changes v1.6:
 *   - PS/2 implementation rewritten based on reference project esp32-bt2ps2
 *     (ps2kb.h / ps2kb.cpp) — direct port, preserves FreeRTOS tasks,
 *     open-drain GPIO, taskENTER_CRITICAL, precise timing constants
 *   - keyHid_send() replaces custom scan code logic
 */


// ── Konfigurace ───────────────────────────────────────────────────────────────
#define BRIDGE_NAME   "BLE-PS2-Bridge"
#define PS2_CLK_PIN   19
#define PS2_DAT_PIN   18
#define NVS_NS        "ble-ps2"
#define NVS_KEY_MAC   "mac"
#define NVS_KEY_TYPE  "type"
#define CONNECT_TRIES 3

// ── Global state ──────────────────────────────────────────────────────────────
static PS2Keyboard    keyboard(PS2_CLK_PIN, PS2_DAT_PIN);

static NimBLEClient*  pClient           = nullptr;
static unsigned long  reconnectAt       = 0;
static int            reconnectFailures = 0;
static bool           reconnectScanActive = false; // scan-before-connect mode

static Preferences    prefs;
static char           savedMAC[18]      = "";
static uint8_t        savedType         = 1;

static uint8_t        prevKeys[6]       = {0};
static uint8_t        prevMod           = 0;

static BLEUUID HID_SERVICE_UUID    ("00001812-0000-1000-8000-00805f9b34fb");
static BLEUUID HID_REPORT_UUID     ("00002a4d-0000-1000-8000-00805f9b34fb");
static BLEUUID BATTERY_SERVICE_UUID("0000180f-0000-1000-8000-00805f9b34fb");
static BLEUUID BATTERY_LEVEL_UUID  ("00002a19-0000-1000-8000-00805f9b34fb");

static int8_t         batteryLevel    = -1;  // -1 = unknown
static volatile bool  statusRequested = false; // set by HID combo, handled in loop()
static NimBLERemoteCharacteristic* pLedChar = nullptr; // HID output report (LED state)

static unsigned long  scanEndAt = 0;
static int            scanCount = 0;

// ── Typematic (key repeat while held) ────────────────────────────────────────
// Same logic as reference project: 500 ms initial delay, then 20 keys/s
#define TYPEMATIC_DELAY_MS  500
#define TYPEMATIC_RATE_MS    50
static unsigned long  typematicNext  = 0;
static bool           typematicArmed = false;
static uint8_t        typematicKey   = 0;

// ── Key name helper for debug output ─────────────────────────────────────────
const char* hidKeyName(uint8_t k) {
  if (k >= 0x04 && k <= 0x1D) {
    static char b[3]; b[0] = 'A' + (k - 0x04); b[1] = 0; return b;
  }
  if (k >= 0x1E && k <= 0x27) {
    static char b[4]; sprintf(b, "D%d", (k == 0x27) ? 0 : (k - 0x1D)); return b;
  }
  switch (k) {
    case 0x28: return "Enter";   case 0x29: return "Esc";
    case 0x2A: return "BkSp";   case 0x2B: return "Tab";
    case 0x2C: return "Space";  case 0x39: return "CapsLk";
    case 0x3A: return "F1";     case 0x3B: return "F2";
    case 0x3C: return "F3";     case 0x3D: return "F4";
    case 0x3E: return "F5";     case 0x3F: return "F6";
    case 0x40: return "F7";     case 0x41: return "F8";
    case 0x42: return "F9";     case 0x43: return "F10";
    case 0x44: return "F11";    case 0x45: return "F12";
    case 0x4C: return "Del";    case 0x52: return "Up";
    case 0x51: return "Down";   case 0x50: return "Left";
    case 0x4F: return "Right";
    default: { static char b[5]; sprintf(b,"%02X",k); return b; }
  }
}

const char* modName(int bit) {
  switch (bit) {
    case 0: return "LCtrl";  case 1: return "LShift";
    case 2: return "LAlt";   case 3: return "LGUI";
    case 4: return "RCtrl";  case 5: return "RShift";
    case 6: return "RAlt";   case 7: return "RGUI";
    default: return "?";
  }
}

// ── HID report → PS/2 ────────────────────────────────────────────────────────
//
// keyboard.keyHid_send() accepts HID keycodes 0x04–0xE7
// Modifier bits mapped to 0xE0–0xE7 (same scheme as reference project)

void processHIDReport(uint8_t* data, size_t len) {
  if (len < 3) return;
  uint8_t  mod  = data[0];
  uint8_t* keys = data + 2;

  Serial.print("[HID] ");
  for (size_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.println();

  // Modifier keys (bits 0–7 → HID 0xE0–0xE7)
  for (int bit = 0; bit < 8; bit++) {
    uint8_t mask = 1 << bit;
    bool wasDown = (prevMod & mask) != 0;
    bool isDown  = (mod     & mask) != 0;
    if (isDown && !wasDown) {
      Serial.printf("[+] %s\n", modName(bit));
      keyboard.keyHid_send(0xE0 + bit, true);
    } else if (!isDown && wasDown) {
      Serial.printf("[-] %s\n", modName(bit));
      keyboard.keyHid_send(0xE0 + bit, false);
    }
  }

  // Released keys
  for (int i = 0; i < 6; i++) {
    if (prevKeys[i] == 0 || prevKeys[i] == 0x01) continue;
    bool found = false;
    for (int j = 0; j < 6; j++) if (keys[j] == prevKeys[i]) { found = true; break; }
    if (!found) {
      Serial.printf("[-] %s\n", hidKeyName(prevKeys[i]));
      keyboard.keyHid_send(prevKeys[i], false);
    }
  }

  // Pressed keys
  for (int i = 0; i < 6; i++) {
    if (keys[i] == 0 || keys[i] == 0x01) continue;
    bool found = false;
    for (int j = 0; j < 6; j++) if (prevKeys[j] == keys[i]) { found = true; break; }
    if (!found) {
      Serial.printf("[+] %s\n", hidKeyName(keys[i]));
      keyboard.keyHid_send(keys[i], true);
    }
  }

  prevMod = mod;
  memcpy(prevKeys, keys, 6);

  // Ctrl+Alt+PrintScreen → request status type-out
  // LCtrl=bit0, LAlt=bit2  (or RCtrl=bit4, RAlt=bit6) + PrintScreen=0x46
  bool ctrl = (mod & 0x01) || (mod & 0x10);
  bool alt  = (mod & 0x04) || (mod & 0x40);
  bool prtsc = false;
  for (int i = 0; i < 6; i++) if (keys[i] == 0x46) { prtsc = true; break; }
  if (ctrl && alt && prtsc) statusRequested = true;

  // Typematic: find first held non-modifier key (skip CapsLock 0x39)
  typematicKey = 0;
  for (int i = 0; i < 6; i++) {
    if (keys[i] && keys[i] != 0x01 && keys[i] != 0x39) {
      typematicKey = keys[i];
      break;
    }
  }
  if (typematicKey) {
    typematicArmed = false;           // reset — wait for initial delay
    typematicNext  = millis() + TYPEMATIC_DELAY_MS;
  } else {
    typematicArmed = false;
    typematicNext  = 0;
  }
}

void notifyCallback(NimBLERemoteCharacteristic* pChar,
                    uint8_t* pData, size_t length, bool isNotify) {
  processHIDReport(pData, length);
}

// ── BLE scan ──────────────────────────────────────────────────────────────────
class MyScanCallbacks : public NimBLEScanCallbacks {
  void onDiscovered(const NimBLEAdvertisedDevice* dev) override {
    // Reconnect scan: detect saved keyboard and trigger immediate connect
    if (reconnectScanActive && strlen(savedMAC) > 0) {
      if (dev->getAddress().toString() == std::string(savedMAC)) {
        Serial.println("[SCAN] Keyboard found — connecting...");
        NimBLEDevice::getScan()->stop();
        reconnectScanActive = false;
        reconnectAt = millis() + 50; // connect almost immediately
        return;
      }
    }
    // Manual scan: print named devices
    if (!dev->haveName() || dev->getName().length() == 0) return;
    Serial.printf("  %-28s  %s\n",
      dev->getName().c_str(), dev->getAddress().toString().c_str());
    scanCount++;
  }
  void onScanEnd(const NimBLEScanResults& r, int reason) override {
    if (reconnectScanActive) {
      // Keyboard not seen in this scan window — retry scan
      reconnectScanActive = false;
      reconnectAt = millis() + 500; // short pause then scan again
    }
  }
};

// ── BLE connection ────────────────────────────────────────────────────────────
bool tryConnect(NimBLEAddress addr) {
  if (pClient) {
    if (pClient->isConnected()) pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
    delay(200);
  }

  pClient = NimBLEDevice::createClient();
  pClient->setConnectionParams(6, 12, 0, 3200);
  pClient->setConnectTimeout(12);

  if (!pClient->connect(addr)) {
    NimBLEDevice::deleteClient(pClient); pClient = nullptr;
    return false;
  }

  if (pClient->secureConnection()) Serial.println("[BLE] Bonding OK");

  NimBLERemoteService* svc = pClient->getService(HID_SERVICE_UUID);
  if (!svc) {
    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient); pClient = nullptr;
    return false;
  }

  // Fetch only HID_REPORT_UUID characteristics — much faster than getCharacteristics(true)
  int subs = 0;
  const std::vector<NimBLERemoteCharacteristic*>& chars =
      svc->getCharacteristics(&HID_REPORT_UUID);
  for (NimBLERemoteCharacteristic* c : chars) {
    if (c->canNotify()) {
      c->subscribe(true, notifyCallback);
      subs++;
    }
  }

  if (subs == 0) {
    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient); pClient = nullptr;
    return false;
  }

  // Find HID Output Report characteristic for LED state forwarding
  // It shares UUID 0x2a4d with input reports but is writable, not notifiable
  pLedChar = nullptr;
  for (NimBLERemoteCharacteristic* c : chars) {
    if (c->canWrite()) {
      pLedChar = c;
      break;
    }
  }
  if (pLedChar) Serial.println("[BLE] LED output char found");

  // Subscribe to battery level notifications if available
  batteryLevel = -1;
  NimBLERemoteService* batSvc = pClient->getService(BATTERY_SERVICE_UUID);
  if (batSvc) {
    NimBLERemoteCharacteristic* batChar = batSvc->getCharacteristic(BATTERY_LEVEL_UUID);
    if (batChar) {
      if (batChar->canRead()) {
        std::string val = batChar->readValue();
        if (!val.empty()) batteryLevel = (int8_t)(uint8_t)val[0];
      }
      if (batChar->canNotify())
        batChar->subscribe(true, [](NimBLERemoteCharacteristic*, uint8_t* d, size_t l, bool) {
          if (l > 0) { batteryLevel = d[0]; } // updated silently, shown on status request
        });
    }
  }

  Serial.printf("[BLE] Bridge active — %d HID report(s)\n", subs);
  return true;
}

bool connectWithRetry(NimBLEAddress addr, int tries) {
  for (int i = 1; i <= tries; i++) {
    Serial.printf("[BLE] Attempt %d/%d: %s\n", i, tries, addr.toString().c_str());
    if (tryConnect(addr)) return true;
    Serial.println("[BLE] Failed.");
    if (i < tries) delay(1500);
  }
  return false;
}

// ── NVS ──────────────────────────────────────────────────────────────────────
bool loadSavedKeyboard() {
  prefs.begin(NVS_NS, true);
  String mac = prefs.getString(NVS_KEY_MAC, "");
  savedType  = prefs.getUChar(NVS_KEY_TYPE, 1);
  prefs.end();
  if (mac.length() == 0) return false;
  strncpy(savedMAC, mac.c_str(), sizeof(savedMAC));
  return true;
}

void saveKeyboard(NimBLEAddress addr) {
  strncpy(savedMAC, addr.toString().c_str(), sizeof(savedMAC));
  savedType = addr.getType();
  prefs.begin(NVS_NS, false);
  prefs.putString(NVS_KEY_MAC, savedMAC);
  prefs.putUChar(NVS_KEY_TYPE, savedType);
  prefs.end();
  Serial.printf("[NVS] Saved: %s (type %d)\n", savedMAC, savedType);
}

void forgetKeyboard() {
  reconnectAt = 0; reconnectFailures = 0;
  if (pClient) {
    if (pClient->isConnected()) pClient->disconnect();
    NimBLEDevice::deleteClient(pClient); pClient = nullptr;
  }
  pLedChar = nullptr;
  NimBLEDevice::deleteAllBonds();
  prefs.begin(NVS_NS, false);
  prefs.remove(NVS_KEY_MAC); prefs.remove(NVS_KEY_TYPE);
  prefs.end();
  memset(savedMAC, 0, sizeof(savedMAC));
  memset(prevKeys, 0, 6); prevMod = 0;
  Serial.println("[FORGET] Cleared. Use: scan  then  connect <mac>");
}

// ── Serial console commands ───────────────────────────────────────────────────
void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  scan              Scan BLE 10s");
  Serial.println("  connect <mac>     Connect and save (3 attempts)");
  Serial.println("  forget            Clear saved keyboard");
  Serial.println("  status            Show connection status");
  Serial.println("  help              Show this help\n");
}

void cmdScan() {
  if (scanEndAt != 0) { Serial.println("[SCAN] Already running..."); return; }
  NimBLEDevice::getScan()->stop(); delay(200);
  scanCount = 0;
  Serial.println("[SCAN] Scanning 10s — put keyboard into PAIRING MODE");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true); scan->setInterval(50); scan->setWindow(45);
  scan->start(0, false);
  scanEndAt = millis() + 10000;
}

void cmdConnect(String mac) {
  mac.trim();
  if (mac.length() < 11) { Serial.println("[ERR] connect xx:xx:xx:xx:xx:xx"); return; }
  if (scanEndAt != 0) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; }
  reconnectAt = 0; reconnectFailures = 0;
  NimBLEDevice::deleteAllBonds();
  NimBLEAddress addr(mac.c_str(), 1);
  if (connectWithRetry(addr, CONNECT_TRIES))
    saveKeyboard(pClient->getPeerAddress());
  else
    Serial.println("[BLE] All attempts failed.");
}

// Build status as a String — shared by Serial output and PS/2 type-out
String buildStatus() {
  bool conn = pClient && pClient->isConnected();
  String s = "";
  s += "\n--- BLE-PS2 Bridge status ---\n";
  s += String("BLE:      ") + (conn ? "CONNECTED" : "DISCONNECTED") + "\n";
  if (conn) {
    NimBLEConnInfo info = pClient->getConnInfo();
    s += String("Peer MAC: ") + pClient->getPeerAddress().toString().c_str() + "\n";
    s += String("RSSI:     ") + pClient->getRssi() + " dBm\n";
    s += String("Interval: ") + info.getConnInterval() + " ms\n";
    s += String("Encrypt:  ") + (info.isEncrypted() ? "yes" : "no") + "\n";
    s += String("Bonded:   ") + (info.isBonded()    ? "yes" : "no") + "\n";
    if (batteryLevel >= 0)
      s += String("Battery:  ") + batteryLevel + "%\n";
    else
      s += "Battery:  unknown\n";
  } else {
    if (reconnectScanActive)
      s += "Scanning for keyboard...\n";
    else if (reconnectAt)
      s += String("Reconnect in: ") + (long)(reconnectAt - millis()) + " ms\n";
    s += String("Failures: ") + reconnectFailures + "\n";
  }
  s += "\n";
  s += String("NVS MAC:  ") + (strlen(savedMAC) ? savedMAC : "(none)") + "\n";
  s += String("Bonds:    ") + NimBLEDevice::getNumBonds() + "\n";
  s += "-----------------------------\n";
  return s;
}

void cmdStatus() {
  Serial.print(buildStatus());
}

// Type status text via PS/2 (called from loop() when statusRequested flag is set)
void typeStatusViaPS2() {
  String s = buildStatus();
  for (int i = 0; i < (int)s.length(); i++) {
    char c = s[i];
    if (c == '\n') {
      keyboard.keyHid_send(0x28, true);  // Enter press
      delay(20);
      keyboard.keyHid_send(0x28, false); // Enter release
    } else if (c == ' ') {
      keyboard.keyHid_send(0x2C, true);
      delay(20);
      keyboard.keyHid_send(0x2C, false);
    } else if (c == '-') {
      keyboard.keyHid_send(0x56, true);  // HID minus (international KB) or 0x2D
      delay(20);
      keyboard.keyHid_send(0x56, false);
    } else if (c == '%') {
      // Shift+5
      keyboard.keyHid_send(0xE1, true);
      keyboard.keyHid_send(0x22, true);
      delay(20);
      keyboard.keyHid_send(0x22, false);
      keyboard.keyHid_send(0xE1, false);
    } else if (c >= 'a' && c <= 'z') {
      keyboard.keyHid_send(0x04 + (c - 'a'), true);
      delay(20);
      keyboard.keyHid_send(0x04 + (c - 'a'), false);
    } else if (c >= 'A' && c <= 'Z') {
      keyboard.keyHid_send(0xE1, true);  // LShift
      keyboard.keyHid_send(0x04 + (c - 'A'), true);
      delay(20);
      keyboard.keyHid_send(0x04 + (c - 'A'), false);
      keyboard.keyHid_send(0xE1, false);
    } else if (c >= '0' && c <= '9') {
      uint8_t k = (c == '0') ? 0x27 : 0x1E + (c - '1');
      keyboard.keyHid_send(k, true);
      delay(20);
      keyboard.keyHid_send(k, false);
    } else if (c == '.') {
      keyboard.keyHid_send(0x37, true);
      delay(20);
      keyboard.keyHid_send(0x37, false);
    } else if (c == ':') {
      keyboard.keyHid_send(0xE1, true);
      keyboard.keyHid_send(0x33, true);
      delay(20);
      keyboard.keyHid_send(0x33, false);
      keyboard.keyHid_send(0xE1, false);
    }
    delay(30); // inter-character gap
  }
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n'); line.trim();
  if (line.length() == 0) return;
  if      (line.equalsIgnoreCase("scan"))    cmdScan();
  else if (line.startsWith("connect ") ||
           line.startsWith("connect\t"))     cmdConnect(line.substring(8));
  else if (line.equalsIgnoreCase("forget"))  forgetKeyboard();
  else if (line.equalsIgnoreCase("status"))  cmdStatus();
  else if (line.equalsIgnoreCase("help") ||
           line.equalsIgnoreCase("?"))       printHelp();
  else Serial.printf("[CMD] Unknown command: '%s'\n", line.c_str());
}

// ── BLE daemon task ──────────────────────────────────────────────────────────
// Same approach as reference project: dedicated task that periodically
// checks the connection and triggers reconnect if the keyboard is lost.
// Runs independently of loop() — reacts even when loop() blocks on BLE.

void bleDaemonTask(void* arg) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(3000)); // check every 3 s

    if (strlen(savedMAC) == 0) continue;                     // no saved keyboard
    if (pClient && pClient->isConnected()) continue;          // all good
    if (reconnectAt != 0) continue;                           // reconnect already scheduled by loop()

    // Keyboard lost and loop() hasn't caught it yet — schedule reconnect
    Serial.println("[DAEMON] Keyboard lost, scheduling reconnect...");
    memset(prevKeys, 0, 6);
    prevMod        = 0;
    typematicKey   = 0;
    typematicArmed = false;
    typematicNext  = 0;
    // Start scan — wait for keyboard to advertise before connecting
    if (!reconnectScanActive) {
      reconnectScanActive = true;
      NimBLEDevice::getScan()->start(5000, false);
      Serial.println("[SCAN] Waiting for keyboard to advertise...");
    }
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  // Disable WiFi — not needed, saves ~20 mA
  esp_wifi_stop();
  esp_wifi_deinit();

  Serial.println("\n================================");
  Serial.println("  BLE -> PS/2 AT Bridge  v1.6");
  Serial.printf ("  CLK=GPIO%d  DAT=GPIO%d\n", PS2_CLK_PIN, PS2_DAT_PIN);
  Serial.println("================================");

  // keyboard.begin() configures GPIO, sends BAT 0xAA, starts FreeRTOS tasks
  keyboard.begin();

  // BLE
  NimBLEDevice::init(BRIDGE_NAME);
  NimBLEDevice::setPower(9);
  NimBLEDevice::setSecurityAuth(true, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::getScan()->setScanCallbacks(new MyScanCallbacks(), false);

  // BLE daemon task — monitors connection independently of loop()
  xTaskCreatePinnedToCore(bleDaemonTask, "ble_daemon", 4096, nullptr, 1, nullptr, 0);

  if (loadSavedKeyboard()) {
    Serial.printf("[NVS] Saved keyboard: %s\n", savedMAC);
    reconnectAt = millis() + 500;
  } else {
    Serial.println("[NVS] No saved keyboard.");
    Serial.println("      Use 'scan' then 'connect <mac>'.");
  }
  printHelp();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  handleSerial();

  // Konec scanu
  if (scanEndAt && millis() >= scanEndAt) {
    scanEndAt = 0;
    NimBLEDevice::getScan()->stop();
    if (scanCount == 0)
      Serial.println("[SCAN] No devices found. Try again.");
    else
      Serial.printf("[SCAN] Done — %d devices found. Use: connect <mac>\n", scanCount);
  }

  // Disconnection detection
  if (pClient && !pClient->isConnected() && strlen(savedMAC) > 0 && reconnectAt == 0) {
    Serial.println("[BLE] Keyboard disconnected.");
    NimBLEDevice::deleteClient(pClient); pClient = nullptr;
    memset(prevKeys, 0, 6); prevMod = 0;
    reconnectFailures = 0;
    // Start a scan to detect when keyboard starts advertising
    reconnectScanActive = true;
    NimBLEDevice::getScan()->start(5000, false); // 5s scan window
    Serial.println("[SCAN] Waiting for keyboard to advertise...");
  }

  // Reconnect — triggered by scan callback when keyboard is seen
  if (reconnectAt && millis() >= reconnectAt && strlen(savedMAC) > 0) {
    reconnectAt = 0;
    if (!pClient || !pClient->isConnected()) {
      if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; }
      NimBLEAddress addr(savedMAC, savedType);
      if (tryConnect(addr)) {
        reconnectFailures = 0;
      } else {
        reconnectFailures++;
        Serial.printf("[BLE] Connect failed (%dx) — scanning again...\n", reconnectFailures);
        // Back to scan mode — wait for keyboard to advertise again
        reconnectScanActive = true;
        NimBLEDevice::getScan()->start(5000, false);
      }
    }
  }

  // Forward LED state to BLE keyboard when PS/2 host updates LEDs
  if (pendingLedMask != 0xFF && pLedChar) {
    uint8_t led = pendingLedMask;
    pendingLedMask = 0xFF;
    pLedChar->writeValue(&led, 1, false);
    Serial.printf("[LED] BLE LED mask 0x%02X\n", led);
  }

  // Status type-out via PS/2 (triggered by Ctrl+Alt+PrintScreen)
  if (statusRequested) {
    statusRequested = false;
    typeStatusViaPS2();
  }

  // Typematic repeat
  if (typematicKey && typematicNext && millis() >= typematicNext) {
    if (!typematicArmed) {
      // First repeat after TYPEMATIC_DELAY_MS — switch to fast interval
      typematicArmed = true;
      typematicNext  = millis() + TYPEMATIC_RATE_MS;
    } else {
      typematicNext = millis() + TYPEMATIC_RATE_MS;
    }
    keyboard.keyHid_send(typematicKey, true);
  }

  delay(5);
}
