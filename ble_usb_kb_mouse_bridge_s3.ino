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

// ESP32 Arduino core does not define MOUSE_BACK / MOUSE_FORWARD
// USBHIDMouse button byte: bit0=L bit1=R bit2=M bit3=Back bit4=Forward
#ifndef MOUSE_BACK
  #define MOUSE_BACK    0x08
#endif
#ifndef MOUSE_FORWARD
  #define MOUSE_FORWARD 0x10
#endif

// ── BLE ───────────────────────────────────────────────────────────────────────
#include <NimBLEDevice.h>
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
static int           scanCount = 0;
static unsigned long scanEndAt = 0;

// Preferences
static Preferences prefs;

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

  Serial.print("[KB] ");
  for (size_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.println();

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
      Serial.println("[SCAN] Keyboard found — connecting...");
      NimBLEDevice::getScan()->stop();
      kbReconnectScan = false;
      kbReconnectAt   = millis() + 50;
    }
    // Reconnect detection for mouse
    if (mouseReconnectScan && strlen(mouseMAC) && addr == std::string(mouseMAC)) {
      Serial.println("[SCAN] Mouse found — connecting...");
      NimBLEDevice::getScan()->stop();
      mouseReconnectScan = false;
      mouseReconnectAt   = millis() + 50;
    }
  }

  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (kbReconnectScan || mouseReconnectScan) return;
    if (!dev->haveServiceUUID() || !dev->isAdvertisingService(HID_SVC_UUID)) return;
    const char* app  = dev->haveAppearance() ? bleAppName(dev->getAppearance()) : "HID";
    const char* name = (dev->haveName() && dev->getName().length()) ? dev->getName().c_str() : "-";
    Serial.printf("  #%-2d  %-17s  %-10s  %4d dBm  %s\n",
      scanCount+1, dev->getAddress().toString().c_str(), app, dev->getRSSI(), name);
    scanCount++;
  }

  void onScanEnd(const NimBLEScanResults&, int) override {
    // If a reconnect scan ended without finding the device, retry after delay
    if (kbReconnectScan)    { kbReconnectScan    = false; kbReconnectAt    = millis() + 500; }
    if (mouseReconnectScan) { mouseReconnectScan = false; mouseReconnectAt = millis() + 500; }
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
  Serial.printf("[KB] Connecting to %s ...\n", addr.toString().c_str());
  pClientKb = NimBLEDevice::createClient();
  pClientKb->setConnectionParams(6, 12, 0, 3200);
  pClientKb->setConnectTimeout(12);

  if (!pClientKb->connect(addr)) {
    NimBLEDevice::deleteClient(pClientKb); pClientKb = nullptr;
    return false;
  }
  if (pClientKb->secureConnection()) Serial.println("[KB] Bonding OK");

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
  Serial.printf("[KB] Connected — %d char(s), battery %d%%\n", subs, kbBattery);
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
  Serial.printf("[MOUSE] Connecting to %s ...\n", addr.toString().c_str());
  pClientMouse = NimBLEDevice::createClient();
  pClientMouse->setConnectionParams(6, 12, 0, 3200);
  pClientMouse->setConnectTimeout(12);

  if (!pClientMouse->connect(addr)) {
    NimBLEDevice::deleteClient(pClientMouse); pClientMouse = nullptr;
    return false;
  }
  if (pClientMouse->secureConnection()) Serial.println("[MOUSE] Bonding OK");

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
    Serial.printf("[MOUSE] handle=0x%04X  ID=%-3d  Type=%d  %s\n",
      c->getHandle(), rptId, rptType, skip ? "skip" : "SUBSCRIBE");
    if (skip) continue;
    c->subscribe(true, mouseNotifyCallback); subs++;
    if (g_mouseHandleCount < MAX_MOUSE_HANDLES)
      g_mouseHandles[g_mouseHandleCount++] = c->getHandle();
  }

  // Fallback if all filtered out
  if (!subs) {
    Serial.println("[MOUSE] Fallback: subscribing all notifiable chars");
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

  Serial.printf("[MOUSE] Connected — %d char(s), battery %d%%\n", subs, mouseBattery);
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
  Serial.println("\n========================================");
  Serial.println("  BLE -> USB HID Keyboard + Mouse Bridge");
  Serial.println("  ESP32-S3");
  Serial.println("========================================");
  Serial.println("  scan                 Scan BLE HID devices (10s)");
  Serial.println("  connect kb <mac>     Connect BLE keyboard");
  Serial.println("  connect mouse <mac>  Connect BLE mouse");
  Serial.println("  forget kb            Forget keyboard");
  Serial.println("  forget mouse         Forget mouse");
  Serial.println("  forget all           Forget both + clear settings");
  Serial.println("  scale <1-64>         Mouse DPI divisor (default 4)");
  Serial.println("  flipy                Toggle mouse Y inversion");
  Serial.println("  flipw                Toggle mouse scroll inversion");
  Serial.println("  reportid <0-255>     Mouse Report ID filter (0=auto)");
  Serial.println("  status               Show connection status");
  Serial.println("  help                 Show this list");
  Serial.println("========================================\n");
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n'); line.trim();
  if (!line.length()) return;

  if (line == "help") {
    printHelp();

  } else if (line == "scan") {
    if (scanEndAt) { Serial.println("[SCAN] Already running."); return; }
    NimBLEDevice::getScan()->stop(); delay(100);
    scanCount = 0;
    NimBLEDevice::getScan()->start(0, false);
    scanEndAt = millis() + 10000;
    Serial.println("[SCAN] 10s — HID devices only (put devices into pairing mode)");
    Serial.println("  #    MAC                Type        RSSI    Name");
    Serial.println("  ─────────────────────────────────────────────────");

  } else if (line.startsWith("connect kb ")) {
    String mac = line.substring(11); mac.trim();
    if (mac.length() < 17) { Serial.println("[ERR] connect kb xx:xx:xx:xx:xx:xx"); return; }
    if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; }
    kbReconnectAt = 0; kbReconnectFails = 0;
    NimBLEDevice::deleteAllBonds();
    bool ok = false;
    NimBLEAddress addr(mac.c_str(), 1);
    for (int i = 0; i < CONNECT_TRIES && !ok; i++) {
      Serial.printf("[KB] Attempt %d/%d\n", i+1, CONNECT_TRIES);
      ok = tryConnectKb(addr);
    }
    if (ok) { saveKb(pClientKb->getPeerAddress()); Serial.printf("[NVS] Keyboard saved: %s\n", kbMAC); }
    else    { Serial.println("[KB] Connection failed."); }

  } else if (line.startsWith("connect mouse ")) {
    String mac = line.substring(14); mac.trim();
    if (mac.length() < 17) { Serial.println("[ERR] connect mouse xx:xx:xx:xx:xx:xx"); return; }
    if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; }
    mouseReconnectAt = 0; mouseReconnectFails = 0;
    bool ok = false;
    NimBLEAddress addr(mac.c_str(), 1);
    for (int i = 0; i < CONNECT_TRIES && !ok; i++) {
      Serial.printf("[MOUSE] Attempt %d/%d\n", i+1, CONNECT_TRIES);
      ok = tryConnectMouse(addr);
    }
    if (ok) { saveMouse(pClientMouse->getPeerAddress()); Serial.printf("[NVS] Mouse saved: %s\n", mouseMAC); saveMouseSettings(); }
    else    { Serial.println("[MOUSE] Connection failed."); }

  } else if (line == "forget kb") {
    if (pClientKb && pClientKb->isConnected()) pClientKb->disconnect();
    prefs.begin(NVS_NS, false); prefs.remove("kb-mac"); prefs.remove("kb-type"); prefs.end();
    memset(kbMAC, 0, sizeof(kbMAC)); kbReconnectAt = 0;
    memset(prevKeys, 0, 6); prevMod = 0; hidKb.releaseAll();
    Serial.println("[NVS] Keyboard forgotten.");

  } else if (line == "forget mouse") {
    if (pClientMouse && pClientMouse->isConnected()) pClientMouse->disconnect();
    prefs.begin(NVS_NS, false); prefs.remove("ms-mac"); prefs.remove("ms-type"); prefs.end();
    memset(mouseMAC, 0, sizeof(mouseMAC)); mouseReconnectAt = 0;
    portENTER_CRITICAL(&g_mux); g_accX=g_accY=g_accW=0; g_buttons=0; g_dirty=false; portEXIT_CRITICAL(&g_mux);
    hidMouse.release(MOUSE_LEFT); hidMouse.release(MOUSE_RIGHT); hidMouse.release(MOUSE_MIDDLE); hidMouse.release(MOUSE_BACK); hidMouse.release(MOUSE_FORWARD);
    Serial.println("[NVS] Mouse forgotten.");

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
    Serial.println("[NVS] All forgotten — settings reset to defaults.");

  } else if (line.startsWith("scale ")) {
    int n = line.substring(6).toInt();
    if (n >= 1 && n <= 64) { g_scaleDivisor = n; saveMouseSettings(); Serial.printf("[CFG] Scale 1/%d saved.\n", n); }
    else Serial.println("[CFG] scale 1-64");

  } else if (line == "flipy") {
    g_flipY = !g_flipY; saveMouseSettings();
    Serial.printf("[CFG] FlipY: %s saved.\n", g_flipY ? "ON" : "OFF");

  } else if (line == "flipw") {
    g_flipW = !g_flipW; saveMouseSettings();
    Serial.printf("[CFG] FlipW: %s saved.\n", g_flipW ? "ON" : "OFF");

  } else if (line.startsWith("reportid ")) {
    int n = line.substring(9).toInt();
    if (n >= 0 && n <= 255) { g_filterReportId = (uint8_t)n; saveMouseSettings(); Serial.printf("[CFG] ReportID %d saved. Reconnect mouse to apply.\n", n); }
    else Serial.println("[CFG] reportid 0-255");

  } else if (line == "status") {
    Serial.print(buildStatus());

  } else {
    Serial.printf("[ERR] Unknown: %s\n", line.c_str());
  }
}

// =============================================================================
//  BLE DAEMON TASK — monitors both connections
// =============================================================================

void bleDaemonTask(void* arg) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Keyboard reconnect
    if (strlen(kbMAC) && !(pClientKb && pClientKb->isConnected()) && !kbReconnectAt) {
      Serial.println("[DAEMON] Keyboard lost — scheduling reconnect...");
      memset(prevKeys, 0, 6); prevMod = 0; typematicKey = 0; typematicArmed = false;
      hidKb.releaseAll();
      if (!kbReconnectScan && !mouseReconnectScan) {
        kbReconnectScan = true;
        NimBLEDevice::getScan()->start(5000, false);
        Serial.println("[SCAN] Waiting for keyboard...");
      } else {
        kbReconnectAt = millis() + 5000;  // retry later if scan already running
      }
    }

    // Mouse reconnect
    if (strlen(mouseMAC) && !(pClientMouse && pClientMouse->isConnected()) && !mouseReconnectAt) {
      Serial.println("[DAEMON] Mouse lost — scheduling reconnect...");
      portENTER_CRITICAL(&g_mux); g_accX=g_accY=g_accW=0; g_buttons=0; g_dirty=false; portEXIT_CRITICAL(&g_mux);
      g_prevButtons = 0;
      hidMouse.release(MOUSE_LEFT); hidMouse.release(MOUSE_RIGHT); hidMouse.release(MOUSE_MIDDLE); hidMouse.release(MOUSE_BACK); hidMouse.release(MOUSE_FORWARD);
      if (!kbReconnectScan && !mouseReconnectScan) {
        mouseReconnectScan = true;
        NimBLEDevice::getScan()->start(5000, false);
        Serial.println("[SCAN] Waiting for mouse...");
      } else {
        mouseReconnectAt = millis() + 5000;
      }
    }
  }
}

// =============================================================================
//  SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // USB HID
  hidKb.begin();
  hidMouse.begin();
  USB.begin();
  delay(1000);  // wait for USB enumeration

  Serial.println("\n========================================");
  Serial.println("  BLE -> USB HID Keyboard + Mouse Bridge");
  Serial.println("  ESP32-S3");
  Serial.println("========================================");

  // BLE
  NimBLEDevice::init(BRIDGE_NAME);
  NimBLEDevice::setPower(9);
  NimBLEDevice::setSecurityAuth(true, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::getScan()->setScanCallbacks(new MyScanCallbacks(), false);
  NimBLEDevice::getScan()->setActiveScan(true);
  NimBLEDevice::getScan()->setInterval(50);
  NimBLEDevice::getScan()->setWindow(45);

  xTaskCreatePinnedToCore(bleDaemonTask, "ble_daemon", 4096, nullptr, 1, nullptr, 0);

  // Load saved devices and settings
  loadSavedDevices();
  loadMouseSettings();

  if (strlen(kbMAC)) {
    Serial.printf("[NVS] Saved keyboard: %s\n", kbMAC);
    kbReconnectAt = millis() + 1500;
  } else {
    Serial.println("[NVS] No keyboard. Use: scan → connect kb <mac>");
  }
  if (strlen(mouseMAC)) {
    Serial.printf("[NVS] Saved mouse: %s  scale=1/%d flipy=%s flipw=%s rid=%d\n",
      mouseMAC, (int)g_scaleDivisor, g_flipY?"on":"off", g_flipW?"on":"off", (int)g_filterReportId);
    mouseReconnectAt = millis() + 2000;  // stagger keyboard and mouse connect
  } else {
    Serial.println("[NVS] No mouse. Use: scan → connect mouse <mac>");
  }

  printHelp();
}

// =============================================================================
//  LOOP
// =============================================================================

void loop() {
  handleSerial();

  // ── Scan end ─────────────────────────────────────────────────────────────
  if (scanEndAt && millis() >= scanEndAt) {
    scanEndAt = 0;
    NimBLEDevice::getScan()->stop();
    Serial.printf("[SCAN] Done — %d device(s).\n", scanCount);
  }

  // ── Keyboard disconnection ───────────────────────────────────────────────
  if (pClientKb && !pClientKb->isConnected() && strlen(kbMAC) && !kbReconnectAt) {
    Serial.println("[KB] Disconnected.");
    NimBLEDevice::deleteClient(pClientKb); pClientKb = nullptr;
    memset(prevKeys, 0, 6); prevMod = 0; hidKb.releaseAll();
    kbReconnectFails = 0; kbReconnectScan = true;
    NimBLEDevice::getScan()->start(5000, false);
  }

  // ── Mouse disconnection ──────────────────────────────────────────────────
  if (pClientMouse && !pClientMouse->isConnected() && strlen(mouseMAC) && !mouseReconnectAt) {
    Serial.println("[MOUSE] Disconnected.");
    NimBLEDevice::deleteClient(pClientMouse); pClientMouse = nullptr;
    portENTER_CRITICAL(&g_mux); g_accX=g_accY=g_accW=0; g_buttons=0; g_dirty=false; portEXIT_CRITICAL(&g_mux);
    g_prevButtons = 0;
    hidMouse.release(MOUSE_LEFT); hidMouse.release(MOUSE_RIGHT); hidMouse.release(MOUSE_MIDDLE); hidMouse.release(MOUSE_BACK); hidMouse.release(MOUSE_FORWARD);
    mouseReconnectFails = 0; mouseReconnectScan = true;
    NimBLEDevice::getScan()->start(5000, false);
  }

  // ── Keyboard reconnect ───────────────────────────────────────────────────
  if (kbReconnectAt && millis() >= kbReconnectAt && strlen(kbMAC)) {
    kbReconnectAt = 0;
    if (!pClientKb || !pClientKb->isConnected()) {
      if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; }
      if (tryConnectKb(NimBLEAddress(kbMAC, kbType))) {
        kbReconnectFails = 0;
      } else {
        Serial.printf("[KB] Connect failed (%dx)\n", ++kbReconnectFails);
        kbReconnectScan = true;
        NimBLEDevice::getScan()->start(5000, false);
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
        Serial.printf("[MOUSE] Connect failed (%dx)\n", ++mouseReconnectFails);
        mouseReconnectScan = true;
        NimBLEDevice::getScan()->start(5000, false);
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
