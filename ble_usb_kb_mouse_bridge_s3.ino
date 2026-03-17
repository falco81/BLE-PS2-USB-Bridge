/*
 * BLE → USB HID Keyboard + Mouse Bridge  (ESP32-S3)
 * ===================================================
 * Hardware : ESP32-S3 N16R8
 * Framework: Arduino ESP32 core 3.x, NimBLE-Arduino 2.x
 * IDE      : Arduino IDE 2.x
 *
 * Features:
 *   - Connects to a BLE HID keyboard AND a BLE HID mouse simultaneously
 *   - Forwards keyboard as USB HID keyboard
 *   - Forwards mouse as USB HID mouse (movement, buttons, scroll wheel)
 *   - Native USB on ESP32-S3 — single USB-C cable to PC
 *   - Typematic key repeat
 *   - Independent auto-reconnect for each device
 *   - NVS — both devices remembered across reboots
 *   - Serial console (115200 baud via UART/CH340 port)
 *
 * ARDUINO IDE SETTINGS:
 *   Board   : ESP32S3 Dev Module
 *   USB Mode: USB-OTG (TinyUSB)   ← REQUIRED
 *   Upload  : UART0 / USB CDC
 *
 * Wiring:
 *   USB-C cable → PC (USB HID output + power)
 *   No extra components needed.
 *
 * Commands (Serial Monitor 115200 baud):
 *   scan               Scan BLE HID devices 10s (shows Keyboard/Mouse)
 *   connect kb <mac>   Connect BLE keyboard and save
 *   connect mouse <mac> Connect BLE mouse and save
 *   forget kb          Forget saved keyboard
 *   forget mouse       Forget saved mouse
 *   forget all         Forget both devices
 *   scale <1-64>       Mouse movement divisor (default 4)
 *   flipy              Toggle mouse Y inversion
 *   flipw              Toggle mouse scroll wheel inversion
 *   reportid <N>       Mouse BLE Report ID filter (0=auto)
 *   status             Show connection status and settings
 *   help               Show this list
 */

// ── USB HID (native ESP32-S3) ─────────────────────────────────────────────────
#include "USB.h"
#include "USBHIDKeyboard.h"
#include "USBHIDMouse.h"
#include "USBCDC.h"

// ESP32 Arduino core does not define MOUSE_BACK / MOUSE_FORWARD
// USBHIDMouse button byte: bit0=L bit1=R bit2=M bit3=Back bit4=Forward
#ifndef MOUSE_BACK
  #define MOUSE_BACK    0x08
#endif
#ifndef MOUSE_FORWARD
  #define MOUSE_FORWARD 0x10
#endif

// ── BLE ───────────────────────────────────────────────────────────────────────
// Increase NimBLE host task stack — combo firmware runs two simultaneous BLE
// clients (keyboard + mouse) which requires more stack than the default 4096.
// Without this the host task overflows on Core 0 during NimBLEDevice::init().
#ifndef CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE
  #define CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE 8192
#endif
#include <NimBLEDevice.h>
#include <set>
#include <Preferences.h>

// ── Konfigurace ───────────────────────────────────────────────────────────────
#define BRIDGE_NAME         "BLE-USB-Bridge"
#define NVS_NS              "ble-usb"
#define CONNECT_TRIES       3
#define KEEPALIVE_MS        3000

// Typematic
#define TYPEMATIC_DELAY_MS  500
#define TYPEMATIC_RATE_MS    50

// ── BLE UUID ──────────────────────────────────────────────────────────────────
static BLEUUID HID_SVC_UUID ("00001812-0000-1000-8000-00805f9b34fb");
static BLEUUID HID_RPT_UUID ("00002a4d-0000-1000-8000-00805f9b34fb");
static BLEUUID BAT_SVC_UUID ("0000180f-0000-1000-8000-00805f9b34fb");
static BLEUUID BAT_LVL_UUID ("00002a19-0000-1000-8000-00805f9b34fb");
static BLEUUID RPT_REF_UUID ("00002908-0000-1000-8000-00805f9b34fb");

// ── USB HID objects ───────────────────────────────────────────────────────────
static USBHIDKeyboard hidKb;
static USBHIDMouse    hidMouse;
static USBCDC* USBSerial = nullptr;  // CDC -- created only when enabled

// ── Console helpers ───────────────────────────────────────────────────────────
//
// USBSerial (CDC) requires CRLF line endings — terminals expect \r\n.
// conPrint/conPrintf translate \n -> \r\n for CDC, send as-is to UART.
//
// CDC line editor: character-by-character read with local echo and
// backspace support so the user sees what they type in PuTTY / any terminal.

static void _usbWrite(const char* s, size_t len) {
  if (!USBSerial) return;
  // Write to CDC — send \n as-is, terminals handle line endings
  for (size_t i = 0; i < len; i++) {
    USBSerial->write((uint8_t)s[i]);
  }
}

static void conPrint(const String& s) {
  Serial.print(s);
  if (USBSerial && USBSerial->availableForWrite() > 0) _usbWrite(s.c_str(), s.length());
}
static void conPrintf(const char* fmt, ...) {
  char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  Serial.print(buf);
  if (USBSerial && USBSerial->availableForWrite() > 0) _usbWrite(buf, strlen(buf));
}

// CDC line editor state
static String _usbLineBuf = "";
static bool   _usbWasConnected = false;  // track CDC connect state for flush

// Read one complete line from USBSerial with echo + backspace support.
// Returns true and sets `line` when Enter is pressed, false otherwise.
static bool _usbReadLine(String& line) {
  while (USBSerial->available()) {
    char c = (char)USBSerial->read();
    if (c == '\r' || c == '\n') {
      if (_usbLineBuf.length() > 0) {
        USBSerial->write("\r\n");   // echo newline
        line = _usbLineBuf;
        _usbLineBuf = "";
        return true;
      }
      USBSerial->write("\r\n");
    } else if (c == 127 || c == '\b') {   // DEL / Backspace
      if (_usbLineBuf.length() > 0) {
        _usbLineBuf.remove(_usbLineBuf.length() - 1);
        USBSerial->write("\b \b");  // erase character on terminal
      }
    } else if (c >= 32) {
      _usbLineBuf += c;
      USBSerial->write((uint8_t)c);  // local echo
    }
  }
  return false;
}

// =============================================================================
//  KEYBOARD — BLE state
// =============================================================================

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

// Keyboard HID state
static uint8_t prevKeys[6] = {0};
static uint8_t prevMod     = 0;

// Typematic
static unsigned long typematicNext  = 0;
static bool          typematicArmed = false;
static uint8_t       typematicKey   = 0;
static uint8_t       typematicMod   = 0;

// Keyboard hotkey flags
static volatile bool statusRequested  = false;
static volatile bool batteryRequested = false;

// =============================================================================
//  MOUSE — BLE state
// =============================================================================

static NimBLEClient* pClientMouse       = nullptr;
static bool          mouseReconnectScan = false;
static unsigned long mouseReconnectAt   = 0;
static int           mouseReconnectFails = 0;
static char          mouseMAC[18]       = "";
static uint8_t       mouseType          = 1;
static int           mouseBattery       = -1;
static unsigned long mouseKeepaliveAt   = 0;
static NimBLERemoteCharacteristic* pMouseBatChar = nullptr;

// Mouse settings (saved to NVS)
static volatile int  g_scaleDivisor = 4;
static volatile bool g_flipY        = false;
static volatile bool g_flipW        = false;
static volatile uint8_t g_filterReportId = 0;

// Mouse accumulator (BLE callback → loop)
static portMUX_TYPE   g_mux      = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t g_accX   = 0;
static volatile int32_t g_accY   = 0;
static volatile int32_t g_accW   = 0;
static volatile uint8_t g_buttons = 0;   // bit0=L bit1=R bit2=M
static volatile bool    g_dirty  = false;
static uint8_t g_prevButtons     = 0;

// Mouse HID report handle filter
#define MAX_MOUSE_HANDLES 8
static uint16_t g_mouseHandles[MAX_MOUSE_HANDLES];
static int      g_mouseHandleCount = 0;

static bool isMouseHandle(uint16_t h) {
  for (int i = 0; i < g_mouseHandleCount; i++)
    if (g_mouseHandles[i] == h) return true;
  return false;
}

// Debug (throttled)
static unsigned long g_lastMovePrint = 0;
static int32_t  g_dbgX = 0, g_dbgY = 0, g_dbgW = 0;
static uint32_t g_dbgPkts = 0;

// Scan
static int           scanCount    = 0;
static unsigned long scanEndAt    = 0;
static volatile bool isManualScan = false;  // true = user-initiated scan vs reconnect scan
static std::set<std::string> scanSeen;          // dedup MAC addresses during manual scan

// Preferences
static Preferences prefs;
static bool g_cdcEnabled = false;  // USB CDC -- default off, saved to NVS

// =============================================================================
//  KEYBOARD — USB HID helpers
// =============================================================================

void usbApplyModifiers(uint8_t mod) {
  for (int bit = 0; bit < 8; bit++) {
    uint8_t mask    = 1 << bit;
    uint8_t keycode = 0xE0 + bit;
    bool was = (prevMod & mask) != 0;
    bool is  = (mod     & mask) != 0;
    if (is  && !was) hidKb.pressRaw(keycode);
    if (!is &&  was) hidKb.releaseRaw(keycode);
  }
}

void usbKeyDown(uint8_t k) { if (k && k != 0x01) hidKb.pressRaw(k); }
void usbKeyUp  (uint8_t k) { if (k && k != 0x01) hidKb.releaseRaw(k); }

const char* hidKeyName(uint8_t k) {
  if (k >= 0x04 && k <= 0x1D) { static char b[3]; b[0]='A'+(k-0x04); b[1]=0; return b; }
  if (k >= 0x1E && k <= 0x27) { static char b[4]; sprintf(b,"D%d",(k==0x27)?0:(k-(uint8_t)0x1D)); return b; }
  switch(k) {
    case 0x28: return "Enter"; case 0x29: return "Esc";  case 0x2A: return "BkSp";
    case 0x2B: return "Tab";   case 0x2C: return "Space";case 0x39: return "CapsLk";
    case 0x3A: return "F1";    case 0x3B: return "F2";   case 0x3C: return "F3";
    case 0x3D: return "F4";    case 0x3E: return "F5";   case 0x3F: return "F6";
    case 0x40: return "F7";    case 0x41: return "F8";   case 0x42: return "F9";
    case 0x43: return "F10";   case 0x44: return "F11";  case 0x45: return "F12";
    case 0x4C: return "Del";   case 0x52: return "Up";   case 0x51: return "Down";
    case 0x50: return "Left";  case 0x4F: return "Right";
    default: { static char b[5]; sprintf(b,"%02X",k); return b; }
  }
}
const char* modName(int b) {
  switch(b) {
    case 0: return "LCtrl"; case 1: return "LShift"; case 2: return "LAlt";
    case 3: return "LGUI";  case 4: return "RCtrl";  case 5: return "RShift";
    case 6: return "RAlt";  case 7: return "RGUI";   default: return "?";
  }
}

// =============================================================================
//  KEYBOARD — HID report processing
// =============================================================================

void processKbReport(uint8_t* data, size_t len) {
  if (len < 2) return;
  uint8_t mod = data[0];

  // Auto-detect reserved byte:
  //   len==8 && data[1]==0x00 → standard [mod][0x00][key1..key6]
  //   otherwise              → compact  [mod][key1..key6]  (reserved omitted)
  bool hasReserved = (len == 8 && data[1] == 0x00);
  uint8_t* keys  = hasReserved ? data + 2 : data + 1;
  size_t   nkeys = hasReserved ? len - 2  : len - 1;
  if (nkeys > 6) nkeys = 6;

  conPrint("[KB] ");
  for (size_t i = 0; i < len; i++) conPrintf("%02X ", data[i]);
  conPrint("\n");

  if (mod != prevMod) {
    usbApplyModifiers(mod);
    for (int bit = 0; bit < 8; bit++) {
      uint8_t mask = 1 << bit;
      bool was = (prevMod & mask) != 0;
      bool is  = (mod     & mask) != 0;
      if (is  && !was) Serial.printf("[KB+] %s\n", modName(bit));
      if (!is &&  was) Serial.printf("[KB-] %s\n", modName(bit));
    }
  }

  for (int i = 0; i < 6; i++) {
    if (!prevKeys[i] || prevKeys[i] == 0x01) continue;
    bool found = false;
    for (size_t j = 0; j < nkeys; j++) if (keys[j] == prevKeys[i]) { found = true; break; }
    if (!found) { Serial.printf("[KB-] %s\n", hidKeyName(prevKeys[i])); usbKeyUp(prevKeys[i]); }
  }
  for (size_t i = 0; i < nkeys; i++) {
    if (!keys[i] || keys[i] == 0x01) continue;
    bool found = false;
    for (int j = 0; j < 6; j++) if (prevKeys[j] == keys[i]) { found = true; break; }
    if (!found) { Serial.printf("[KB+] %s\n", hidKeyName(keys[i])); usbKeyDown(keys[i]); }
  }

  prevMod = mod;
  memset(prevKeys, 0, 6);
  memcpy(prevKeys, keys, nkeys);

  // LED sync (lock key toggle tracking)
  {
    static uint8_t bleLedMask = 0;
    uint8_t lockKeys[] = { 0x53, 0x39, 0x47 };
    uint8_t ledBits[]  = { 0x01, 0x02, 0x04 };
    for (int k = 0; k < 3; k++) {
      bool isDown = false, wasDown = false;
      for (size_t i = 0; i < nkeys; i++) if (keys[i]     == lockKeys[k]) isDown  = true;
      for (int    i = 0; i < 6;     i++) if (prevKeys[i] == lockKeys[k]) wasDown = true;
      if (isDown && !wasDown && pKbLedChar) {
        bleLedMask ^= ledBits[k];
        pKbLedChar->writeValue(&bleLedMask, 1, false);
        Serial.printf("[LED] 0x%02X\n", bleLedMask);
      }
    }
  }

  // Hotkeys
  bool lctrl  = (mod & 0x01) != 0;
  bool lalt   = (mod & 0x04) != 0;
  bool lshift = (mod & 0x02) != 0;
  bool prtsc  = false;
  for (size_t i = 0; i < nkeys; i++) if (keys[i] == 0x46 || keys[i] == 0x9A) { prtsc = true; break; }
  if (lctrl && lalt && lshift && prtsc)  statusRequested  = true;
  if (lctrl && lalt && !lshift && prtsc) batteryRequested = true;

  // Typematic
  typematicKey = 0; typematicMod = mod;
  for (size_t i = 0; i < nkeys; i++) {
    if (keys[i] && keys[i] != 0x01 && keys[i] != 0x39) { typematicKey = keys[i]; break; }
  }
  if (typematicKey) { typematicArmed = false; typematicNext = millis() + TYPEMATIC_DELAY_MS; }
  else              { typematicArmed = false; typematicNext = 0; }
}

void kbNotifyCallback(NimBLERemoteCharacteristic* pChar,
                      uint8_t* pData, size_t length, bool isNotify) {
  processKbReport(pData, length);
}

// =============================================================================
//  MOUSE — BLE HID report parsing
// =============================================================================
//
//  Supported formats (payload WITHOUT Report ID):
//  [3B]  btn | dx8 | dy8
//  [4B]  btn | dx8 | dy8 | wheel
//  [5B]  btn | dx8 | dy8 | wheel | hwheel
//  [7B]  Logitech 12-bit packed (MX Master 2/3, G502, M650):
//        btn | extra | X_lo | X_hi|Y_lo | Y_hi | wheel | hwheel
//
//  Button bits (g_buttons):
//    bit0 = Left       bit1 = Right    bit2 = Middle
//    bit3 = Back       bit4 = Forward  bit5 = Extra (gesture/other)
//
//  For 7B Logitech:
//    d[0] bits 0-4 = L R M Back Forward
//    d[1] bits 0-x = extra buttons (gesture etc.)

static inline int16_t sign12(int32_t v) {
  return (v & 0x800) ? (int16_t)(v - 0x1000) : (int16_t)v;
}

static bool parseBLEReport(const uint8_t* d, size_t len,
                             int16_t& dx, int16_t& dy,
                             int8_t& wheel, uint8_t& buttons)
{
  if (len < 3 || len > 10) return false;
  if (d[0] == 0xFF) return false;  // Logitech vendor specific
  wheel = 0;
  if (len <= 5) {
    // Standard format: only 3 buttons in byte[0]
    buttons = d[0] & 0x07;
    dx = (int8_t)d[1]; dy = (int8_t)d[2];
    if (len >= 4) wheel = (int8_t)d[3];
    return true;
  }
  if (len == 7) {
    // Logitech 12-bit packed — byte[0] has 5 buttons, byte[1] has extra buttons
    // bit0=L bit1=R bit2=M bit3=Back bit4=Forward
    buttons = d[0] & 0x1F;
    // Map any bit in extra byte (d[1]) to bit5
    if (d[1] & 0xFF) buttons |= 0x20;
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

// =============================================================================
//  MOUSE — notify callback
// =============================================================================

static void mouseNotifyCallback(NimBLERemoteCharacteristic* pChar,
                                 uint8_t* pData, size_t length, bool isNotify)
{
  if (g_mouseHandleCount > 0 && !isMouseHandle(pChar->getHandle())) return;

  int16_t dx = 0, dy = 0; int8_t dw = 0; uint8_t btns = 0;
  if (!parseBLEReport(pData, length, dx, dy, dw, btns)) {
    Serial.printf("[MOUSE] unknown len=%d %02X %02X %02X %02X\n",
      (int)length, pData[0], length>1?pData[1]:0,
      length>2?pData[2]:0, length>3?pData[3]:0);
    return;
  }

  portENTER_CRITICAL(&g_mux);
    uint8_t prev = g_buttons;
    g_accX += dx; g_accY += dy; g_accW += dw;
    g_buttons = btns; g_dirty = true;
  portEXIT_CRITICAL(&g_mux);

  if (btns != prev)
    Serial.printf("[BTN] L=%d R=%d M=%d\n", btns&1, (btns>>1)&1, (btns>>2)&1);
}

// =============================================================================
//  MOUSE — USB HID output
// =============================================================================

static void processMouseMovement() {
  if (!g_dirty && g_buttons == g_prevButtons) return;

  int32_t ax, ay, aw;
  uint8_t btns;
  portENTER_CRITICAL(&g_mux);
    ax = g_accX; ay = g_accY; aw = g_accW;
    btns = g_buttons; g_dirty = false;
  portEXIT_CRITICAL(&g_mux);

  int div = (g_scaleDivisor < 1) ? 1 : g_scaleDivisor;
  int32_t sx = ax/div, remX = ax - sx*div;
  int32_t sy = ay/div, remY = ay - sy*div;
  int32_t sw = aw;  // wheel not scaled

  portENTER_CRITICAL(&g_mux);
    g_accX = remX; g_accY = remY; g_accW = 0;
  portEXIT_CRITICAL(&g_mux);

  if (g_flipY) sy = -sy;
  if (g_flipW) sw = -sw;

  if (sx == 0 && sy == 0 && sw == 0 && btns == g_prevButtons) return;

  // Send movement in chunks of ±127 (int8_t range)
  do {
    int8_t px = (int8_t)constrain(sx, -127, 127);
    int8_t py = (int8_t)constrain(sy, -127, 127);
    int8_t pw = (int8_t)constrain(sw, -127, 127);

    // Button deltas — L R M Back Forward Extra
    // USB HID: MOUSE_LEFT=0x01 MOUSE_RIGHT=0x02 MOUSE_MIDDLE=0x04
    //          MOUSE_BACK=0x08 MOUSE_FORWARD=0x10
    struct { uint8_t bit; uint8_t usbBtn; const char* name; } btnMap[] = {
      { 0x01, MOUSE_LEFT,    "L" },
      { 0x02, MOUSE_RIGHT,   "R" },
      { 0x04, MOUSE_MIDDLE,  "M" },
      { 0x08, MOUSE_BACK,    "Back" },
      { 0x10, MOUSE_FORWARD, "Fwd" },
      // bit5 (extra/gesture) — no standard USB mapping, skip
    };
    for (auto& b : btnMap) {
      if ((btns & b.bit) != (g_prevButtons & b.bit)) {
        if (btns & b.bit) { hidMouse.press(b.usbBtn);   Serial.printf("[BTN+] %s\n", b.name); }
        else              { hidMouse.release(b.usbBtn);  Serial.printf("[BTN-] %s\n", b.name); }
      }
    }
    g_prevButtons = btns;

    hidMouse.move(px, py, pw);
    g_dbgX += px; g_dbgY += py; g_dbgW += pw; g_dbgPkts++;
    sx -= px; sy -= py; sw -= pw;
    if (!sx && !sy && !sw) break;
  } while (sx || sy || sw);

  // Throttled debug log
  unsigned long now = millis();
  if (now - g_lastMovePrint >= 500 && g_dbgPkts > 0) {
    Serial.printf("[MOUSE] pkts=%lu X=%d Y=%d W=%d btn=L%d R%d M%d Bk%d Fw%d Ex%d\n",
      g_dbgPkts, (int)g_dbgX, (int)g_dbgY, (int)g_dbgW,
      btns&1, (btns>>1)&1, (btns>>2)&1, (btns>>3)&1, (btns>>4)&1, (btns>>5)&1);
    g_dbgX = g_dbgY = g_dbgW = 0; g_dbgPkts = 0;
    g_lastMovePrint = now;
  }
}

// =============================================================================
//  MOUSE — NVS settings
// =============================================================================

static void saveMouseSettings() {
  prefs.begin(NVS_NS, false);
  prefs.putInt  ("ms-scale", g_scaleDivisor);
  prefs.putBool ("ms-flipy", g_flipY);
  prefs.putBool ("ms-flipw", g_flipW);
  prefs.putUChar("ms-rid",   g_filterReportId);
  prefs.end();
}

static void loadMouseSettings() {
  prefs.begin(NVS_NS, true);
  g_scaleDivisor   = prefs.getInt  ("ms-scale", 4);
  g_flipY          = prefs.getBool ("ms-flipy", false);
  g_flipW          = prefs.getBool ("ms-flipw", false);
  g_filterReportId = prefs.getUChar("ms-rid",   0);
  prefs.end();
  if (g_scaleDivisor < 1 || g_scaleDivisor > 64) g_scaleDivisor = 4;
}

// =============================================================================
//  BLE SCAN
// =============================================================================

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
    // Reconnect detection for keyboard
    if (kbReconnectScan && strlen(kbMAC) && addr == std::string(kbMAC)) {
      conPrint("[SCAN] Keyboard found — connecting..." "\n");
      NimBLEDevice::getScan()->stop();
      kbReconnectScan = false;
      kbReconnectAt   = millis() + 50;
    }
    // Reconnect detection for mouse
    if (mouseReconnectScan && strlen(mouseMAC) && addr == std::string(mouseMAC)) {
      conPrint("[SCAN] Mouse found — connecting..." "\n");
      NimBLEDevice::getScan()->stop();
      mouseReconnectScan = false;
      mouseReconnectAt   = millis() + 50;
    }
  }

  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (!isManualScan) return;  // only print during user-initiated scan
    if (!dev->haveServiceUUID() || !dev->isAdvertisingService(HID_SVC_UUID)) return;
    std::string addr = dev->getAddress().toString();
    if (scanSeen.count(addr)) return;  // already listed
    scanSeen.insert(addr);
    const char* app  = dev->haveAppearance() ? bleAppName(dev->getAppearance()) : "HID";
    // Store name in local string — getName() may return a temporary, c_str() on a
    // temporary is a dangling pointer by the time conPrintf uses it
    std::string nameStr = (dev->haveName() && dev->getName().length()) ? dev->getName() : "-";
    conPrintf("  #%-2d  %-17s  %-10s  %4d dBm  %s\n",
      scanCount+1, addr.c_str(), app, dev->getRSSI(), nameStr.c_str());
    scanCount++;
  }

  void onScanEnd(const NimBLEScanResults&, int) override {
    // If a reconnect scan ended without finding the device, retry after delay
    if (kbReconnectScan)    { kbReconnectScan    = false; kbReconnectAt    = millis() + 200; }
    if (mouseReconnectScan) { mouseReconnectScan = false; mouseReconnectAt = millis() + 200; }
  }
};

// =============================================================================
//  BLE CONNECT — keyboard
// =============================================================================

bool tryConnectKb(NimBLEAddress addr) {
  if (pClientKb) {
    if (pClientKb->isConnected()) pClientKb->disconnect();
    NimBLEDevice::deleteClient(pClientKb);
    pClientKb = nullptr;
    delay(200);
  }
  conPrintf("[KB] Connecting to %s ...\n", addr.toString().c_str());
  pClientKb = NimBLEDevice::createClient();
  pClientKb->setConnectionParams(6, 12, 0, 3200);
  pClientKb->setConnectTimeout(12);

  if (!pClientKb->connect(addr)) {
    NimBLEDevice::deleteClient(pClientKb); pClientKb = nullptr;
    return false;
  }
  if (pClientKb->secureConnection()) conPrint("[KB] Bonding OK" "\n");

  NimBLERemoteService* svc = pClientKb->getService(HID_SVC_UUID);
  if (!svc) {
    pClientKb->disconnect(); NimBLEDevice::deleteClient(pClientKb); pClientKb = nullptr;
    return false;
  }

  int subs = 0;
  const std::vector<NimBLERemoteCharacteristic*>& chars = svc->getCharacteristics(&HID_RPT_UUID);
  for (NimBLERemoteCharacteristic* c : chars) {
    if (c->canNotify()) { c->subscribe(true, kbNotifyCallback); subs++; }
  }
  if (!subs) {
    pClientKb->disconnect(); NimBLEDevice::deleteClient(pClientKb); pClientKb = nullptr;
    return false;
  }

  pKbLedChar = nullptr;
  for (NimBLERemoteCharacteristic* c : chars) {
    if (c->canWrite()) { pKbLedChar = c; break; }
  }

  kbBattery = -1; pKbBatChar = nullptr;
  NimBLERemoteService* bs = pClientKb->getService(BAT_SVC_UUID);
  if (bs) {
    NimBLERemoteCharacteristic* bc = bs->getCharacteristic(BAT_LVL_UUID);
    if (bc) {
      if (bc->canRead()) {
        std::string v = bc->readValue();
        if (!v.empty()) kbBattery = (int)(uint8_t)v[0];
        pKbBatChar = bc;
      }
      if (bc->canNotify())
        bc->subscribe(true, [](NimBLERemoteCharacteristic*, uint8_t* d, size_t l, bool) {
          if (l > 0) kbBattery = (int)d[0];
        });
    }
  }
  kbKeepaliveAt = millis() + KEEPALIVE_MS;
  conPrintf("[KB] Connected — %d char(s), battery %d%%\n", subs, kbBattery);
  memset(prevKeys, 0, 6); prevMod = 0;
  return true;
}

// =============================================================================
//  BLE CONNECT — mouse
// =============================================================================

bool tryConnectMouse(NimBLEAddress addr) {
  if (pClientMouse) {
    if (pClientMouse->isConnected()) pClientMouse->disconnect();
    NimBLEDevice::deleteClient(pClientMouse);
    pClientMouse = nullptr;
    delay(200);
  }
  conPrintf("[MOUSE] Connecting to %s ...\n", addr.toString().c_str());
  pClientMouse = NimBLEDevice::createClient();
  pClientMouse->setConnectionParams(6, 12, 0, 3200);
  pClientMouse->setConnectTimeout(12);

  if (!pClientMouse->connect(addr)) {
    NimBLEDevice::deleteClient(pClientMouse); pClientMouse = nullptr;
    return false;
  }
  if (pClientMouse->secureConnection()) conPrint("[MOUSE] Bonding OK" "\n");

  NimBLERemoteService* svc = pClientMouse->getService(HID_SVC_UUID);
  if (!svc) {
    pClientMouse->disconnect(); NimBLEDevice::deleteClient(pClientMouse); pClientMouse = nullptr;
    return false;
  }

  // Use Report Reference descriptor (0x2908) to identify Input reports
  // Subscribe only Input type (type=1), skip boot protocol (id=0)
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
    conPrintf("[MOUSE] handle=0x%04X  ID=%-3d  Type=%d  %s\n",
      c->getHandle(), rptId, rptType, skip ? "skip" : "SUBSCRIBE");
    if (skip) continue;
    c->subscribe(true, mouseNotifyCallback); subs++;
    if (g_mouseHandleCount < MAX_MOUSE_HANDLES)
      g_mouseHandles[g_mouseHandleCount++] = c->getHandle();
  }

  // Fallback if all filtered out
  if (!subs) {
    conPrint("[MOUSE] Fallback: subscribing all notifiable chars" "\n");
    for (NimBLERemoteCharacteristic* c : chars) {
      if (!c->canNotify()) continue;
      c->subscribe(true, mouseNotifyCallback); subs++;
      if (g_mouseHandleCount < MAX_MOUSE_HANDLES)
        g_mouseHandles[g_mouseHandleCount++] = c->getHandle();
    }
  }
  if (!subs) {
    pClientMouse->disconnect(); NimBLEDevice::deleteClient(pClientMouse); pClientMouse = nullptr;
    return false;
  }

  mouseBattery = -1; pMouseBatChar = nullptr;
  NimBLERemoteService* bs = pClientMouse->getService(BAT_SVC_UUID);
  if (bs) {
    NimBLERemoteCharacteristic* bc = bs->getCharacteristic(BAT_LVL_UUID);
    if (bc) {
      if (bc->canRead()) {
        std::string v = bc->readValue();
        if (!v.empty()) mouseBattery = (int)(uint8_t)v[0];
        if (bc->canRead()) pMouseBatChar = bc;
      }
      if (bc->canNotify())
        bc->subscribe(true, [](NimBLERemoteCharacteristic*, uint8_t* d, size_t l, bool) {
          if (l > 0) mouseBattery = (int)d[0];
        });
    }
  }
  mouseKeepaliveAt = millis() + KEEPALIVE_MS;

  portENTER_CRITICAL(&g_mux);
    g_accX = g_accY = g_accW = 0; g_buttons = 0; g_dirty = false;
  portEXIT_CRITICAL(&g_mux);
  g_prevButtons = 0;

  conPrintf("[MOUSE] Connected — %d char(s), battery %d%%\n", subs, mouseBattery);
  return true;
}

// =============================================================================
//  NVS — load / save
// =============================================================================

static void loadSavedDevices() {
  prefs.begin(NVS_NS, true);
  String km = prefs.getString("kb-mac", "");
  kbType = prefs.getUChar("kb-type", 1);
  String mm = prefs.getString("ms-mac", "");
  mouseType = prefs.getUChar("ms-type", 1);
  prefs.end();
  if (km.length()) { strncpy(kbMAC,    km.c_str(), 17); kbMAC[17]    = 0; }
  if (mm.length()) { strncpy(mouseMAC, mm.c_str(), 17); mouseMAC[17] = 0; }
}

static void saveKb(const NimBLEAddress& a) {
  strncpy(kbMAC, a.toString().c_str(), 17); kbMAC[17] = 0;
  kbType = (uint8_t)a.getType();
  prefs.begin(NVS_NS, false);
  prefs.putString("kb-mac", kbMAC);
  prefs.putUChar ("kb-type", kbType);
  prefs.end();
}

static void saveMouse(const NimBLEAddress& a) {
  strncpy(mouseMAC, a.toString().c_str(), 17); mouseMAC[17] = 0;
  mouseType = (uint8_t)a.getType();
  prefs.begin(NVS_NS, false);
  prefs.putString("ms-mac", mouseMAC);
  prefs.putUChar ("ms-type", mouseType);
  prefs.end();
}

// =============================================================================
//  STATUS
// =============================================================================

static String buildStatus() {
  String s = "\n--- BLE-USB Bridge status ---\n";

  bool kbConn    = pClientKb    && pClientKb->isConnected();
  bool mouseConn = pClientMouse && pClientMouse->isConnected();

  s += "--- Keyboard ---\n";
  s += String("BLE:      ") + (kbConn ? "CONNECTED" : "disconnected") + "\n";
  s += String("MAC:      ") + (strlen(kbMAC) ? kbMAC : "(none)") + "\n";
  if (kbConn) {
    s += String("RSSI:     ") + pClientKb->getRssi() + " dBm\n";
    s += String("Battery:  ") + (kbBattery >= 0 ? (String(kbBattery) + "%") : "unknown") + "\n";
  } else if (kbReconnectScan) s += "Scanning...\n";

  s += "\n--- Mouse ---\n";
  s += String("BLE:      ") + (mouseConn ? "CONNECTED" : "disconnected") + "\n";
  s += String("MAC:      ") + (strlen(mouseMAC) ? mouseMAC : "(none)") + "\n";
  if (mouseConn) {
    s += String("RSSI:     ") + pClientMouse->getRssi() + " dBm\n";
    s += String("Battery:  ") + (mouseBattery >= 0 ? (String(mouseBattery) + "%") : "unknown") + "\n";
  } else if (mouseReconnectScan) s += "Scanning...\n";
  s += String("Scale:    1/") + g_scaleDivisor + "\n";
  s += String("FlipY:    ") + (g_flipY ? "yes" : "no") + "\n";
  s += String("FlipW:    ") + (g_flipW ? "yes" : "no") + "\n";
  s += String("ReportID: ") + g_filterReportId + (g_filterReportId ? "" : " (auto)") + "\n";

  s += "-----------------------------\n";
  return s;
}

// Type-out helpers (unchanged from original)
// Helper: type a single character via USB HID (digits, %, letters, space, colon, /)
static void typeChar(char c) {
  if (c >= '0' && c <= '9') {
    uint8_t k = (c == '0') ? (uint8_t)0x27 : (uint8_t)(0x1E + (c - '1'));
    hidKb.pressRaw(k); delay(20); hidKb.releaseRaw(k);
  } else if (c == '%') {
    hidKb.pressRaw(0xE1); hidKb.pressRaw(0x22);
    delay(20); hidKb.releaseRaw(0x22); hidKb.releaseRaw(0xE1);
  } else if (c == ' ') {
    hidKb.pressRaw(0x2C); delay(20); hidKb.releaseRaw(0x2C);
  } else if (c == ':') {
    hidKb.pressRaw(0xE1); hidKb.pressRaw(0x33);
    delay(20); hidKb.releaseRaw(0x33); hidKb.releaseRaw(0xE1);
  } else if (c >= 'A' && c <= 'Z') {
    uint8_t k = (uint8_t)(0x04 + (c - 'A'));
    hidKb.pressRaw(0xE1); hidKb.pressRaw(k);
    delay(20); hidKb.releaseRaw(k); hidKb.releaseRaw(0xE1);
  } else if (c >= 'a' && c <= 'z') {
    uint8_t k = (uint8_t)(0x04 + (c - 'a'));
    hidKb.pressRaw(k); delay(20); hidKb.releaseRaw(k);
  }
  delay(30);
}

static void typeString(const String& s) {
  for (int i = 0; i < (int)s.length(); i++) typeChar(s[i]);
}

// LCtrl+LAlt+PrtSc → type "K: 80% M: 50%" (no Enter)
void typeBatteryViaUSB() {
  hidKb.releaseAll(); delay(50);
  String kb = (kbBattery    >= 0) ? String(kbBattery)    + "%" : "?%";
  String ms = (mouseBattery >= 0) ? String(mouseBattery) + "%" : "?%";
  typeString("K: " + kb + " M: " + ms);
  hidKb.releaseAll();
}

void typeStatusViaUSB() {
  hidKb.releaseAll(); delay(50);
  String s = buildStatus();
  for (int i = 0; i < (int)s.length(); i++) {
    char c = s[i];
    if      (c == '\n') { hidKb.pressRaw(0x28); delay(20); hidKb.releaseRaw(0x28); delay(30); }
    else if (c == '-')  { hidKb.pressRaw(0x2D); delay(20); hidKb.releaseRaw(0x2D); delay(30); }
    else if (c == '/')  { hidKb.pressRaw(0x38); delay(20); hidKb.releaseRaw(0x38); delay(30); }
    else if (c == '.')  { hidKb.pressRaw(0x37); delay(20); hidKb.releaseRaw(0x37); delay(30); }
    else                { typeChar(c); }
  }
  hidKb.releaseAll();
}

// =============================================================================
//  SERIAL CONSOLE
// =============================================================================

void printHelp() {
  conPrint("\n========================================\n");
  conPrint("  BLE -> USB HID Keyboard + Mouse Bridge\n");
  conPrint("  ESP32-S3\n");
  conPrint("========================================\n");
  conPrint("  scan                 Scan BLE HID devices (10s)\n");
  conPrint("  connect kb <mac>     Connect BLE keyboard\n");
  conPrint("  connect mouse <mac>  Connect BLE mouse\n");
  conPrint("  forget kb            Forget keyboard\n");
  conPrint("  forget mouse         Forget mouse\n");
  conPrint("  forget all           Forget both + clear settings\n");
  conPrint("  scale <1-64>         Mouse DPI divisor (default 4)\n");
  conPrint("  flipy                Toggle mouse Y inversion\n");
  conPrint("  flipw                Toggle mouse scroll inversion\n");
  conPrint("  reportid <0-255>     Mouse Report ID filter (0=auto)\n");
  conPrint("  status               Show connection status\n");
  conPrint("  help                 Show this list\n");
  Serial.println("  cdc on/off           Enable/disable USB CDC (UART only)\n"
                 "                       Reboot required to apply.");
  conPrint("========================================\n\n");
}

void handleSerial() {
  String line;
  bool got = false;
  bool uart_cmd = false;  // true = command came from UART
  if (Serial.available()) {
    line = Serial.readStringUntil('\n'); got = true; uart_cmd = true;
  } else if (g_cdcEnabled && _usbReadLine(line)) {
    got = true;
  }
  if (!got) return;
  line.trim();
  if (!line.length()) return;

  if (line == "help") {
    printHelp();

  } else if (line == "scan") {
    if (scanEndAt) { conPrint("[SCAN] Already running.\n"); return; }
    NimBLEDevice::getScan()->stop(); delay(100);
    // Suspend reconnect scans during manual scan — onResult filters them out otherwise
    kbReconnectScan    = false;
    mouseReconnectScan = false;
    isManualScan = true;
    scanCount = 0;
    scanSeen.clear();
    NimBLEDevice::getScan()->start(0, false);
    scanEndAt = millis() + 10000;
    conPrint("[SCAN] 10s — HID devices only (put devices into pairing mode)\n");
    conPrint("  #    MAC                Type        RSSI    Name\n");
    conPrint("  -------------------------------------------------\n");

  } else if (line.startsWith("connect kb ")) {
    String mac = line.substring(11); mac.trim();
    if (mac.length() < 17) { conPrint("[ERR] connect kb xx:xx:xx:xx:xx:xx" "\n"); return; }
    if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; isManualScan = false; }
    kbReconnectAt = 0; kbReconnectFails = 0;
    { NimBLEAddress a(mac.c_str(), 1); NimBLEDevice::deleteBond(a); }
    delay(100);
    bool ok = false;
    NimBLEAddress addr(mac.c_str(), 1);
    for (int i = 0; i < CONNECT_TRIES && !ok; i++) {
      conPrintf("[KB] Attempt %d/%d\n", i+1, CONNECT_TRIES);
      ok = tryConnectKb(addr);
      if (!ok && i < CONNECT_TRIES - 1) delay(1500);
    }
    if (ok) { saveKb(pClientKb->getPeerAddress()); conPrintf("[NVS] Keyboard saved: %s\n", kbMAC); }
    else    { conPrint("[KB] Connection failed.\n"); }

  } else if (line.startsWith("connect mouse ")) {
    String mac = line.substring(14); mac.trim();
    if (mac.length() < 17) { conPrint("[ERR] connect mouse xx:xx:xx:xx:xx:xx\n"); return; }
    if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; isManualScan = false; }
    mouseReconnectAt = 0; mouseReconnectFails = 0;
    // Delete only this device bond — not all bonds (would disconnect keyboard)
    { NimBLEAddress a(mac.c_str(), 1); NimBLEDevice::deleteBond(a); }
    delay(100);
    NimBLEAddress addr(mac.c_str(), 1);
    bool ok = false;
    int attempt = 0;
    // Keep retrying — mouse may take a few seconds to enter pairing mode
    // Each connect() attempt has a 12s timeout internally
    conPrint("[MOUSE] Connecting — keep mouse in pairing mode...\n");
    while (!ok) {
      attempt++;
      conPrintf("[MOUSE] Attempt %d\n", attempt);
      ok = tryConnectMouse(addr);
      if (!ok) {
        if (attempt >= 20) {
          conPrint("[MOUSE] Giving up after 20 attempts. Try: connect mouse <mac> again.\n");
          mouseReconnectAt = millis() + 10000;
          break;
        }
        // Check if any serial command arrived (user wants to abort)
        if (Serial.available()) { Serial.readStringUntil('\n'); conPrint("[MOUSE] Aborted.\n"); break; }
        if (g_cdcEnabled && USBSerial && USBSerial->available()) { _usbLineBuf = ""; while(USBSerial->available()) USBSerial->read(); conPrint("[MOUSE] Aborted.\n"); break; }
        delay(500);
      }
    }
    if (ok) { saveMouse(pClientMouse->getPeerAddress()); conPrintf("[NVS] Mouse saved: %s\n", mouseMAC); saveMouseSettings(); }

  } else if (line == "forget kb") {
    if (pClientKb && pClientKb->isConnected()) pClientKb->disconnect();
    prefs.begin(NVS_NS, false); prefs.remove("kb-mac"); prefs.remove("kb-type"); prefs.end();
    memset(kbMAC, 0, sizeof(kbMAC)); kbReconnectAt = 0;
    memset(prevKeys, 0, 6); prevMod = 0; hidKb.releaseAll();
    conPrint("[NVS] Keyboard forgotten." "\n");

  } else if (line == "forget mouse") {
    if (pClientMouse && pClientMouse->isConnected()) pClientMouse->disconnect();
    prefs.begin(NVS_NS, false); prefs.remove("ms-mac"); prefs.remove("ms-type"); prefs.end();
    memset(mouseMAC, 0, sizeof(mouseMAC)); mouseReconnectAt = 0;
    portENTER_CRITICAL(&g_mux); g_accX=g_accY=g_accW=0; g_buttons=0; g_dirty=false; portEXIT_CRITICAL(&g_mux);
    hidMouse.release(MOUSE_LEFT); hidMouse.release(MOUSE_RIGHT); hidMouse.release(MOUSE_MIDDLE); hidMouse.release(MOUSE_BACK); hidMouse.release(MOUSE_FORWARD);
    conPrint("[NVS] Mouse forgotten." "\n");

  } else if (line == "forget all") {
    if (pClientKb    && pClientKb->isConnected())    pClientKb->disconnect();
    if (pClientMouse && pClientMouse->isConnected()) pClientMouse->disconnect();
    prefs.begin(NVS_NS, false); prefs.clear(); prefs.end();
    NimBLEDevice::deleteAllBonds();
    memset(kbMAC, 0, sizeof(kbMAC)); memset(mouseMAC, 0, sizeof(mouseMAC));
    kbReconnectAt = mouseReconnectAt = 0;
    g_scaleDivisor = 4; g_flipY = false; g_flipW = false; g_filterReportId = 0;
    memset(prevKeys, 0, 6); prevMod = 0;
    hidKb.releaseAll();
    hidMouse.release(MOUSE_LEFT); hidMouse.release(MOUSE_RIGHT); hidMouse.release(MOUSE_MIDDLE); hidMouse.release(MOUSE_BACK); hidMouse.release(MOUSE_FORWARD);
    conPrint("[NVS] All forgotten — settings reset to defaults." "\n");

  } else if (line.startsWith("scale ")) {
    int n = line.substring(6).toInt();
    if (n >= 1 && n <= 64) { g_scaleDivisor = n; saveMouseSettings(); conPrintf("[CFG] Scale 1/%d saved.\n", n); }
    else conPrint("[CFG] scale 1-64" "\n");

  } else if (line == "flipy") {
    g_flipY = !g_flipY; saveMouseSettings();
    conPrintf("[CFG] FlipY: %s saved.\n", g_flipY ? "ON" : "OFF");

  } else if (line == "flipw") {
    g_flipW = !g_flipW; saveMouseSettings();
    conPrintf("[CFG] FlipW: %s saved.\n", g_flipW ? "ON" : "OFF");

  } else if (line.startsWith("reportid ")) {
    int n = line.substring(9).toInt();
    if (n >= 0 && n <= 255) { g_filterReportId = (uint8_t)n; saveMouseSettings(); conPrintf("[CFG] ReportID %d saved. Reconnect mouse to apply.\n", n); }
    else conPrint("[CFG] reportid 0-255" "\n");

  } else if (line == "status") {
    conPrint(buildStatus());

  } else if (line == "cdc on" || line == "cdc off") {
    if (!uart_cmd) {
      conPrint("[ERR] cdc command only available on UART console.\n");
      return;
    }
    bool enable = (line == "cdc on");
    Preferences p; p.begin(NVS_NS, false);
    p.putBool("cdc-en", enable);
    p.end();
    // Do NOT update g_cdcEnabled at runtime -- USB descriptor is fixed at boot
    // The change takes effect after reboot
    Serial.printf("[CDC] USB serial %s -- saved. Reboot to apply.\n",
                  enable ? "ENABLED" : "DISABLED");
  } else {
    conPrintf("[ERR] Unknown: %s\n", line.c_str());
  }
}

// =============================================================================
//  BLE DAEMON TASK — monitors both connections
// =============================================================================

void bleDaemonTask(void* arg) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Keyboard reconnect — skip if manual scan is in progress
    if (!isManualScan && strlen(kbMAC) && !(pClientKb && pClientKb->isConnected()) && !kbReconnectAt) {
      conPrint("[DAEMON] Keyboard lost — scheduling reconnect...\n");
      memset(prevKeys, 0, 6); prevMod = 0; typematicKey = 0; typematicArmed = false;
      hidKb.releaseAll();
      if (!kbReconnectScan && !mouseReconnectScan) {
        kbReconnectScan = true;
        NimBLEDevice::getScan()->start(3000, false);
        conPrint("[SCAN] Waiting for keyboard...\n");
      } else {
        kbReconnectAt = millis() + 2000;
      }
    }

    // Mouse reconnect — skip if manual scan is in progress
    if (!isManualScan && strlen(mouseMAC) && !(pClientMouse && pClientMouse->isConnected()) && !mouseReconnectAt) {
      conPrint("[DAEMON] Mouse lost — scheduling reconnect...\n");
      portENTER_CRITICAL(&g_mux); g_accX=g_accY=g_accW=0; g_buttons=0; g_dirty=false; portEXIT_CRITICAL(&g_mux);
      g_prevButtons = 0;
      hidMouse.release(MOUSE_LEFT); hidMouse.release(MOUSE_RIGHT); hidMouse.release(MOUSE_MIDDLE); hidMouse.release(MOUSE_BACK); hidMouse.release(MOUSE_FORWARD);
      if (!kbReconnectScan && !mouseReconnectScan) {
        mouseReconnectScan = true;
        NimBLEDevice::getScan()->start(3000, false);
        conPrint("[SCAN] Waiting for mouse...\n");
      } else {
        mouseReconnectAt = millis() + 2000;
      }
    }
  }
}

// =============================================================================
//  SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n========================================");
  Serial.println("  BLE -> USB HID Keyboard + Mouse Bridge");
  Serial.println("  ESP32-S3");
  Serial.println("========================================");

  // ── BLE first — sets up Core 0 NimBLE host task before USB touches Core 0
  NimBLEDevice::init(BRIDGE_NAME);
  NimBLEDevice::setPower(9);
  NimBLEDevice::setSecurityAuth(true, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::getScan()->setScanCallbacks(new MyScanCallbacks(), false);
  NimBLEDevice::getScan()->setActiveScan(true);
  NimBLEDevice::getScan()->setInterval(50);
  NimBLEDevice::getScan()->setWindow(45);

  // Allow BLE host task on Core 0 to fully settle before USB IPC calls start
  delay(200);

  // Read NVS first so we know whether to register CDC in USB descriptor
  {
    Preferences p; p.begin(NVS_NS, true);
    g_cdcEnabled = p.getBool("cdc-en", false);
    p.end();
  }

  // ── USB HID + CDC composite — initialise one by one to reduce IPC burst
  // CDC must be registered BEFORE USB.begin() — the descriptor is fixed at that point
  hidKb.begin();
  delay(50);
  hidMouse.begin();
  delay(50);
  if (g_cdcEnabled) {
    USBSerial = new USBCDC();
    USBSerial->begin();
    delay(50);
  }
  USB.begin();
  delay(1000);  // wait for USB enumeration

  Serial.println("[USB] Enumeration complete.");

  xTaskCreatePinnedToCore(bleDaemonTask, "ble_daemon", 8192, nullptr, 1, nullptr, 0);

  // Load saved devices and settings
  loadSavedDevices();
  loadMouseSettings();

  Serial.println("[NVS] ========== Stored configuration ==========");
  Serial.printf( "[NVS] Keyboard MAC:  %s\n",  strlen(kbMAC)    ? kbMAC    : "(none)");
  Serial.printf( "[NVS] Mouse MAC:     %s\n",  strlen(mouseMAC) ? mouseMAC : "(none)");
  Serial.println("[NVS] --- Mouse settings ---");
  Serial.printf( "[NVS] Scale:         1/%d\n", (int)g_scaleDivisor);
  Serial.printf( "[NVS] FlipY:         %s\n",   g_flipY ? "on" : "off");
  Serial.printf( "[NVS] FlipW:         %s\n",   g_flipW ? "on" : "off");
  Serial.printf( "[NVS] ReportID:      %d%s\n", (int)g_filterReportId,
                  g_filterReportId ? "" : " (auto)");
  Serial.println("[NVS] --- System ---");
  Serial.printf( "[NVS] CDC:           %s\n",   g_cdcEnabled ? "enabled" : "disabled");
  Serial.println("[NVS] ============================================");
  if (strlen(kbMAC))    kbReconnectAt    = millis() + 500;
  if (strlen(mouseMAC)) mouseReconnectAt = millis() + 800;
  if (g_cdcEnabled) Serial.println("[CDC] USB serial enabled.");
  else              Serial.println("[CDC] USB serial disabled. Use 'cdc on' to enable.");

  printHelp();
}

// =============================================================================
//  LOOP
// =============================================================================

void loop() {
  handleSerial();

  // ── Scan end ─────────────────────────────────────────────────────────────
  if (scanEndAt && millis() >= scanEndAt) {
    scanEndAt    = 0;
    isManualScan = false;
    NimBLEDevice::getScan()->stop();
    conPrintf("[SCAN] Done — %d device(s). Use: connect kb/mouse <mac>\n", scanCount);
  }

  // ── Keyboard disconnection ───────────────────────────────────────────────
  if (pClientKb && !pClientKb->isConnected() && strlen(kbMAC) && !kbReconnectAt) {
    conPrint("[KB] Disconnected." "\n");
    NimBLEDevice::deleteClient(pClientKb); pClientKb = nullptr;
    memset(prevKeys, 0, 6); prevMod = 0; hidKb.releaseAll();
    kbReconnectFails = 0; kbReconnectScan = true;
    NimBLEDevice::getScan()->start(3000, false);
  }

  // ── Mouse disconnection ──────────────────────────────────────────────────
  if (pClientMouse && !pClientMouse->isConnected() && strlen(mouseMAC) && !mouseReconnectAt) {
    conPrint("[MOUSE] Disconnected." "\n");
    NimBLEDevice::deleteClient(pClientMouse); pClientMouse = nullptr;
    portENTER_CRITICAL(&g_mux); g_accX=g_accY=g_accW=0; g_buttons=0; g_dirty=false; portEXIT_CRITICAL(&g_mux);
    g_prevButtons = 0;
    hidMouse.release(MOUSE_LEFT); hidMouse.release(MOUSE_RIGHT); hidMouse.release(MOUSE_MIDDLE); hidMouse.release(MOUSE_BACK); hidMouse.release(MOUSE_FORWARD);
    mouseReconnectFails = 0; mouseReconnectScan = true;
    NimBLEDevice::getScan()->start(3000, false);
  }

  // ── Keyboard reconnect ───────────────────────────────────────────────────
  if (kbReconnectAt && millis() >= kbReconnectAt && strlen(kbMAC)) {
    kbReconnectAt = 0;
    if (!pClientKb || !pClientKb->isConnected()) {
      if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; }
      if (tryConnectKb(NimBLEAddress(kbMAC, kbType))) {
        kbReconnectFails = 0;
      } else {
        conPrintf("[KB] Connect failed (%dx)\n", ++kbReconnectFails);
        kbReconnectScan = true;
        NimBLEDevice::getScan()->start(3000, false);
      }
    }
  }

  // ── Mouse reconnect ──────────────────────────────────────────────────────
  if (mouseReconnectAt && millis() >= mouseReconnectAt && strlen(mouseMAC)) {
    mouseReconnectAt = 0;
    if (!pClientMouse || !pClientMouse->isConnected()) {
      if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; }
      if (tryConnectMouse(NimBLEAddress(mouseMAC, mouseType))) {
        mouseReconnectFails = 0;
      } else {
        conPrintf("[MOUSE] Connect failed (%dx)\n", ++mouseReconnectFails);
        mouseReconnectScan = true;
        NimBLEDevice::getScan()->start(3000, false);
      }
    }
  }

  // ── Keyboard keepalive ───────────────────────────────────────────────────
  if (kbKeepaliveAt && millis() >= kbKeepaliveAt && pKbBatChar && pClientKb && pClientKb->isConnected()) {
    kbKeepaliveAt = millis() + KEEPALIVE_MS;
    std::string v = pKbBatChar->readValue();
    if (!v.empty()) kbBattery = (int)(uint8_t)v[0];
  }

  // ── Mouse keepalive ──────────────────────────────────────────────────────
  if (mouseKeepaliveAt && millis() >= mouseKeepaliveAt && pMouseBatChar && pClientMouse && pClientMouse->isConnected()) {
    mouseKeepaliveAt = millis() + KEEPALIVE_MS;
    std::string v = pMouseBatChar->readValue();
    if (!v.empty()) mouseBattery = (int)(uint8_t)v[0];
  }

  // ── Keyboard hotkeys ─────────────────────────────────────────────────────
  if (batteryRequested) {
    batteryRequested = false;
    typematicKey = 0; typematicNext = 0;
    hidKb.releaseAll(); delay(50);
    typeBatteryViaUSB();
  }
  if (statusRequested) {
    statusRequested = false;
    typematicKey = 0; typematicNext = 0;
    hidKb.releaseAll(); delay(50);
    typeStatusViaUSB();
  }

  // ── Typematic ────────────────────────────────────────────────────────────
  if (typematicKey && typematicNext && millis() >= typematicNext) {
    if (!typematicArmed) { typematicArmed = true; typematicNext = millis() + TYPEMATIC_RATE_MS; }
    else                 { typematicNext  = millis() + TYPEMATIC_RATE_MS; }
    for (int bit = 0; bit < 8; bit++)
      if (typematicMod & (1 << bit)) hidKb.pressRaw(0xE0 + bit);
    hidKb.pressRaw(typematicKey);
  }

  // ── Mouse movement → USB HID ─────────────────────────────────────────────
  processMouseMovement();

  delay(1);
}
