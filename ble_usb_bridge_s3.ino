/*
 * BLE → USB HID Keyboard Bridge  (Varianta 2 — ESP32-S3)
 * ========================================================
 * Hardware : ESP32-S3 N16R8
 * Framework: Arduino ESP32 core 3.x, NimBLE-Arduino 2.x
 * IDE      : Arduino IDE 2.x
 *
 * Funkce:
 *   - Connects to a BLE HID keyboard as host
 *   - Forwards keystrokes as a USB HID keyboard
 *   - Native USB on ESP32-S3 (no CH340 / converter needed)
 *   - Typematic (key repeat while held)
 *   - Auto-reconnect after keyboard disconnect / power cycle
 *   - NVS — paired keyboard remembered across reboots
 *   - Serial console on both UART (CH340) and USB CDC (HID port)
 *
 * ARDUINO IDE SETTINGS:
 *   Board  : ESP32S3 Dev Module
 *   USB Mode: USB-OTG (TinyUSB)   ← REQUIRED
 *   Upload : UART0 / USB CDC
 *   Port   : COM port ESP32-S3
 *
 * Wiring:
 *   USB-C (HID port) → PC  — keyboard HID + CDC console
 *   USB-C (UART port) → PC — firmware upload + debug output
 *
 * Console:
 *   Commands accepted on UART (115200 baud) AND on USB CDC virtual
 *   serial port that appears on the HID USB connection.
 *   Debug output (BLE events, key logs) goes to UART only.
 */

// ── USB HID (native ESP32-S3) ────────────────────────────────────────────────
#include "USB.h"
#include "USBHIDKeyboard.h"
#include "USBCDC.h"

// ── BLE ───────────────────────────────────────────────────────────────────────
#include <NimBLEDevice.h>
#include <Preferences.h>

// ── Konfigurace ───────────────────────────────────────────────────────────────
#define BRIDGE_NAME          "BLE-USB-Bridge"
#define NVS_NS               "ble-usb"
#define NVS_KEY_MAC          "mac"
#define NVS_KEY_TYPE         "type"
#define CONNECT_TRIES        3

// Typematic — same values as PS/2 variant
#define TYPEMATIC_DELAY_MS   500   // ms before repeat starts
#define TYPEMATIC_RATE_MS     50   // ms between repeats (20 keys/s)

// ── Global objects ────────────────────────────────────────────────────────────
static USBHIDKeyboard    hidKb;
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
  if (!USBSerial) return false;
  // Flush RX buffer on new CDC connection (DTR assertion sends garbage)
  bool connected = (bool)(*USBSerial);
  if (connected && !_usbWasConnected) {
    delay(50);
    while (USBSerial->available()) USBSerial->read();  // discard garbage
    _usbLineBuf = "";
    _usbWasConnected = true;
  } else if (!connected) {
    _usbWasConnected = false;
    _usbLineBuf = "";
  }
  while (USBSerial->available()) {
    char c = (char)USBSerial->read();
    if (c == '\r' || c == '\n') {
      if (_usbLineBuf.length() > 0) {
        USBSerial->write("\r\n");   // echo newline
        line = _usbLineBuf;
        _usbLineBuf = "";
        return true;
      }
      // empty line — just echo newline
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

static NimBLEClient*     pClient           = nullptr;
static unsigned long     reconnectAt       = 0;
static int               reconnectFailures   = 0;
static bool              reconnectScanActive = false;

static Preferences       prefs;
static bool g_cdcEnabled = false;  // USB CDC -- default off, saved to NVS
static char              savedMAC[18]      = "";
static uint8_t           savedType         = 1;

static uint8_t           prevKeys[6]       = {0};
static uint8_t           prevMod           = 0;

static BLEUUID HID_SERVICE_UUID    ("00001812-0000-1000-8000-00805f9b34fb");
static BLEUUID HID_REPORT_UUID     ("00002a4d-0000-1000-8000-00805f9b34fb");
static BLEUUID BATTERY_SERVICE_UUID("0000180f-0000-1000-8000-00805f9b34fb");
static BLEUUID BATTERY_LEVEL_UUID  ("00002a19-0000-1000-8000-00805f9b34fb");

static int            batteryLevel    = -1;  // -1 = unknown, 0-100 = percent
static volatile bool  statusRequested  = false; // LCtrl+LAlt+LShift+PrtSc
static volatile bool  batteryRequested = false; // LCtrl+LAlt+PrtSc
static NimBLERemoteCharacteristic* pLedChar  = nullptr;
static NimBLERemoteCharacteristic* pBatChar  = nullptr;
static unsigned long  keepaliveAt = 0;
#define KEEPALIVE_MS  3000

static unsigned long     scanEndAt  = 0;
static int               scanCount  = 0;

// Typematic stav
static unsigned long     typematicNext  = 0;
static bool              typematicArmed = false;
static uint8_t           typematicKey   = 0;   // HID keycode of held key
static uint8_t           typematicMod   = 0;   // modifiers at time of hold

// ── USB HID — key transmission ────────────────────────────────────────────────
//
// BLE HID and USB HID use identical keycodes (HID Usage Table).
// Forward the report directly — no translation table needed.
//
// USB HID modifier byte:
//   bit 0 = LCtrl  bit 1 = LShift  bit 2 = LAlt  bit 3 = LGUI
//   bit 4 = RCtrl  bit 5 = RShift  bit 6 = RAlt  bit 7 = RGUI

void usbApplyModifiers(uint8_t mod) {
  // USBHIDKeyboard has no pressModifier/releaseModifier.
  // Modifier keycodes are 0xE0–0xE7 (LCtrl, LShift, LAlt, LGUI, RCtrl, RShift, RAlt, RGUI).
  // Diff against prevMod and press/release individual keys via pressRaw/releaseRaw.
  for (int bit = 0; bit < 8; bit++) {
    uint8_t mask    = 1 << bit;
    uint8_t keycode = 0xE0 + bit;
    bool was = (prevMod & mask) != 0;
    bool is  = (mod     & mask) != 0;
    if (is  && !was) hidKb.pressRaw(keycode);
    if (!is &&  was) hidKb.releaseRaw(keycode);
  }
}

void usbKeyDown(uint8_t keycode) {
  if (keycode == 0 || keycode == 0x01) return;
  hidKb.pressRaw(keycode);
}

void usbKeyUp(uint8_t keycode) {
  if (keycode == 0 || keycode == 0x01) return;
  hidKb.releaseRaw(keycode);
}

// ── Debug helpers ─────────────────────────────────────────────────────────────
const char* hidKeyName(uint8_t k) {
  if (k >= 0x04 && k <= 0x1D) {
    static char b[3]; b[0] = 'A' + (k - 0x04); b[1] = 0; return b;
  }
  if (k >= 0x1E && k <= 0x27) {
    static char b[4]; sprintf(b, "D%d", (k == 0x27) ? 0 : (k - 0x1D)); return b;
  }
  switch (k) {
    case 0x28: return "Enter";  case 0x29: return "Esc";
    case 0x2A: return "BkSp";  case 0x2B: return "Tab";
    case 0x2C: return "Space"; case 0x39: return "CapsLk";
    case 0x3A: return "F1";    case 0x3B: return "F2";
    case 0x3C: return "F3";    case 0x3D: return "F4";
    case 0x3E: return "F5";    case 0x3F: return "F6";
    case 0x40: return "F7";    case 0x41: return "F8";
    case 0x42: return "F9";    case 0x43: return "F10";
    case 0x44: return "F11";   case 0x45: return "F12";
    case 0x4C: return "Del";   case 0x52: return "Up";
    case 0x51: return "Down";  case 0x50: return "Left";
    case 0x4F: return "Right";
    default: { static char b[5]; sprintf(b, "%02X", k); return b; }
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

// ── HID report → USB HID ─────────────────────────────────────────────────────
void processHIDReport(uint8_t* data, size_t len) {
  if (len < 2) return;
  uint8_t mod = data[0];

  // Auto-detect report format:
  //   8 bytes: [mod][reserved=0x00][key1..key6]  — standard HID keyboard
  //   7 bytes: [mod][key1..key6]                 — reserved byte omitted (e.g. esp32_mouse_keyboard)
  // Heuristic: if len==8 and data[1]==0x00, treat as reserved byte present.
  // For any other length, assume no reserved byte so keys start at data[1].
  bool hasReserved = (len == 8 && data[1] == 0x00);
  uint8_t* keys  = hasReserved ? data + 2 : data + 1;
  size_t   nkeys = hasReserved ? len - 2  : len - 1;
  if (nkeys > 6) nkeys = 6;

  conPrint("[HID] ");
  for (size_t i = 0; i < len; i++) conPrintf("%02X ", data[i]);
  conPrint("\n");

  // Modifiers — apply full bitmask at once
  if (mod != prevMod) {
    usbApplyModifiers(mod);
    // Log
    for (int bit = 0; bit < 8; bit++) {
      uint8_t mask = 1 << bit;
      bool was = (prevMod & mask) != 0;
      bool is  = (mod     & mask) != 0;
      if (is  && !was) conPrintf("[+] %s\n", modName(bit));
      if (!is &&  was) conPrintf("[-] %s\n", modName(bit));
    }
  }

  // Released keys
  for (int i = 0; i < 6; i++) {
    if (prevKeys[i] == 0 || prevKeys[i] == 0x01) continue;
    bool found = false;
    for (size_t j = 0; j < nkeys; j++) if (keys[j] == prevKeys[i]) { found = true; break; }
    if (!found) {
      conPrintf("[-] %s\n", hidKeyName(prevKeys[i]));
      usbKeyUp(prevKeys[i]);
    }
  }

  // Pressed keys
  for (size_t i = 0; i < nkeys; i++) {
    if (keys[i] == 0 || keys[i] == 0x01) continue;
    bool found = false;
    for (int j = 0; j < 6; j++) if (prevKeys[j] == keys[i]) { found = true; break; }
    if (!found) {
      conPrintf("[+] %s\n", hidKeyName(keys[i]));
      usbKeyDown(keys[i]);
    }
  }

  prevMod = mod;
  memset(prevKeys, 0, 6);
  memcpy(prevKeys, keys, nkeys);

  // Forward LED state to BLE keyboard on lock key press
  // PC sends USB HID output reports but they are not easily interceptable
  // in Arduino core, so we track lock key toggles ourselves.
  {
    static uint8_t bleLedMask = 0;
    uint8_t lockKeys[] = { 0x53, 0x39, 0x47 }; // NumLock, CapsLock, ScrollLock
    uint8_t ledBits[]  = { 0x01, 0x02, 0x04 }; // BLE HID LED bits
    for (int k = 0; k < 3; k++) {
      bool isDown  = false;
      bool wasDown = false;
      for (size_t i = 0; i < nkeys; i++) if (keys[i]     == lockKeys[k]) isDown  = true;
      for (int    i = 0; i < 6;     i++) if (prevKeys[i] == lockKeys[k]) wasDown = true;
      if (isDown && !wasDown) {  // key just pressed
        bleLedMask ^= ledBits[k];
        if (pLedChar) {
          pLedChar->writeValue(&bleLedMask, 1, false);
          Serial.printf("[LED] BLE LED mask 0x%02X\n", bleLedMask);
        }
      }
    }
  }

  // Hotkey detection — Left modifiers only
  // LCtrl=bit0  LShift=bit1  LAlt=bit2  PrintScreen=0x46
  bool lctrl  = (mod & 0x01) != 0;
  bool lalt   = (mod & 0x04) != 0;
  bool lshift = (mod & 0x02) != 0;
  bool prtsc  = false;
  for (size_t i = 0; i < nkeys; i++) if (keys[i] == 0x46 || keys[i] == 0x9A) { prtsc = true; break; }
  if (lctrl && lalt && lshift && prtsc)  statusRequested  = true;
  if (lctrl && lalt && !lshift && prtsc) batteryRequested = true;

  // Typematic — first held key (skip CapsLock 0x39)
  typematicKey = 0;
  typematicMod = mod;
  for (size_t i = 0; i < nkeys; i++) {
    if (keys[i] && keys[i] != 0x01 && keys[i] != 0x39) {
      typematicKey = keys[i];
      break;
    }
  }
  if (typematicKey) {
    typematicArmed = false;
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
    // Reconnect scan: detect saved keyboard
    if (reconnectScanActive && strlen(savedMAC) > 0) {
      if (dev->getAddress().toString() == std::string(savedMAC)) {
        conPrint("[SCAN] Keyboard found — connecting..." "\n");
        NimBLEDevice::getScan()->stop();
        reconnectScanActive = false;
        reconnectAt = millis() + 50;
        return;
      }
    }
    // Manual scan: print named devices
    if (!dev->haveName() || dev->getName().length() == 0) return;
    conPrintf("  %-30s  %s\n",
      dev->getName().c_str(), dev->getAddress().toString().c_str());
    scanCount++;
  }
  void onScanEnd(const NimBLEScanResults& r, int reason) override {
    if (reconnectScanActive) {
      reconnectScanActive = false;
      reconnectAt = millis() + 500;
    }
  }
};

// ── BLE connection ────────────────────────────────────────────────────────────
bool loadSavedKeyboard() {
  prefs.begin(NVS_NS, true);
  String mac = prefs.getString(NVS_KEY_MAC, "");
  savedType  = prefs.getUChar(NVS_KEY_TYPE, 1);
  prefs.end();
  if (mac.length() == 0) return false;
  strncpy(savedMAC, mac.c_str(), 17); savedMAC[17] = 0;
  return true;
}

void saveKeyboard(const NimBLEAddress& addr) {
  prefs.begin(NVS_NS, false);
  prefs.putString(NVS_KEY_MAC, addr.toString().c_str());
  prefs.putUChar(NVS_KEY_TYPE, (uint8_t)addr.getType());
  prefs.end();
  strncpy(savedMAC, addr.toString().c_str(), 17);
  savedType = (uint8_t)addr.getType();
}

bool tryConnect(NimBLEAddress addr) {
  if (pClient) {
    if (pClient->isConnected()) pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
    delay(200);
  }

  conPrintf("[BLE] Connecting to %s ...\n", addr.toString().c_str());
  pClient = NimBLEDevice::createClient();
  pClient->setConnectionParams(6, 12, 0, 3200);
  pClient->setConnectTimeout(12);

  if (!pClient->connect(addr)) {
    NimBLEDevice::deleteClient(pClient); pClient = nullptr;
    conPrint("[BLE] Connection failed." "\n");
    return false;
  }

  if (pClient->secureConnection()) conPrint("[BLE] Bonding OK" "\n");

  NimBLERemoteService* svc = pClient->getService(HID_SERVICE_UUID);
  if (!svc) {
    conPrint("[BLE] HID service not found." "\n");
    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient); pClient = nullptr;
    return false;
  }

  // Fetch only HID_REPORT_UUID characteristics — faster than getCharacteristics(true)
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
  pLedChar = nullptr;
  for (NimBLERemoteCharacteristic* c : chars) {
    if (c->canWrite()) {
      pLedChar = c;
      break;
    }
  }
  if (pLedChar) conPrint("[BLE] LED output char found" "\n");

  // Battery level
  batteryLevel = -1;
  NimBLERemoteService* batSvc = pClient->getService(BATTERY_SERVICE_UUID);
  if (batSvc) {
    NimBLERemoteCharacteristic* batChar = batSvc->getCharacteristic(BATTERY_LEVEL_UUID);
    if (batChar) {
      if (batChar->canRead()) {
        std::string val = batChar->readValue();
        if (!val.empty()) batteryLevel = (int)(uint8_t)val[0];
        pBatChar = batChar;
      }
      if (batChar->canNotify())
        batChar->subscribe(true, [](NimBLERemoteCharacteristic*, uint8_t* d, size_t l, bool) {
          if (l > 0) { batteryLevel = (int)d[0]; }
        });
    }
  }
  keepaliveAt = millis() + KEEPALIVE_MS;

  conPrintf("[BLE] Bridge active — %d HID report(s)\n", subs);
  return true;
}

// ── Status output ────────────────────────────────────────────────────────────
String buildStatus() {
  bool conn = pClient && pClient->isConnected();
  String s = "";
  s += "\n--- BLE-USB Bridge status ---\n";
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

// Type battery percentage via USB HID (e.g. "78%")
void typeBatteryViaUSB() {
  hidKb.releaseAll();
  delay(50);
  String s = (batteryLevel >= 0) ? (String(batteryLevel) + "%") : "?%";
  for (int i = 0; i < (int)s.length(); i++) {
    char c = s[i];
    if (c >= '0' && c <= '9') {
      uint8_t k = (c == '0') ? 0x27 : 0x1E + (c - '1');
      hidKb.pressRaw(k); delay(20); hidKb.releaseRaw(k); delay(30);
    } else if (c == '%') {
      hidKb.pressRaw(0xE1); hidKb.pressRaw(0x22);
      delay(20);
      hidKb.releaseRaw(0x22); hidKb.releaseRaw(0xE1); delay(30);
    } else if (c == '?') {
      hidKb.pressRaw(0xE1); hidKb.pressRaw(0x38);
      delay(20);
      hidKb.releaseRaw(0x38); hidKb.releaseRaw(0xE1); delay(30);
    }
  }
  hidKb.releaseAll();
}

// Type status via USB HID keyboard (for Ctrl+Alt+PrintScreen in any text field)
void typeStatusViaUSB() {
  hidKb.releaseAll();
  delay(50);
  String s = buildStatus();
  for (int i = 0; i < (int)s.length(); i++) {
    char c = s[i];
    if (c == '\n') {
      hidKb.pressRaw(0x28); delay(20); hidKb.releaseRaw(0x28);
    } else if (c == ' ') {
      hidKb.pressRaw(0x2C); delay(20); hidKb.releaseRaw(0x2C);
    } else if (c == '-') {
      hidKb.pressRaw(0x2D); delay(20); hidKb.releaseRaw(0x2D);
    } else if (c == '%') {
      hidKb.pressRaw(0xE1); hidKb.pressRaw(0x22); delay(20);
      hidKb.releaseRaw(0x22); hidKb.releaseRaw(0xE1);
    } else if (c >= 'a' && c <= 'z') {
      hidKb.pressRaw(0x04 + (c - 'a')); delay(20);
      hidKb.releaseRaw(0x04 + (c - 'a'));
    } else if (c >= 'A' && c <= 'Z') {
      hidKb.pressRaw(0xE1);
      hidKb.pressRaw(0x04 + (c - 'A')); delay(20);
      hidKb.releaseRaw(0x04 + (c - 'A')); hidKb.releaseRaw(0xE1);
    } else if (c >= '0' && c <= '9') {
      uint8_t k = (c == '0') ? 0x27 : 0x1E + (c - '1');
      hidKb.pressRaw(k); delay(20); hidKb.releaseRaw(k);
    } else if (c == '.') {
      hidKb.pressRaw(0x37); delay(20); hidKb.releaseRaw(0x37);
    } else if (c == ':') {
      hidKb.pressRaw(0xE1); hidKb.pressRaw(0x33); delay(20);
      hidKb.releaseRaw(0x33); hidKb.releaseRaw(0xE1);
    }
    delay(30);
  }
  hidKb.releaseAll();
}

// ── Serial konzole ────────────────────────────────────────────────────────────
void printHelp() {
  conPrint("\n--- Commands ---\n");
  conPrint("  scan          BLE scan (10s)\n");
  conPrint("  connect <mac> Connect and save\n");
  conPrint("  forget        Clear paired keyboard\n");
  conPrint("  status        Connection status\n");
  conPrint("  help          Show this list\n");
  Serial.println("  cdc on/off    Enable/disable USB CDC (UART only)\n"                 "  Reboot required to apply.");
  conPrint("---------------\n");
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
  if (line.length() == 0) return;

  if (line == "help") {
    printHelp();
  } else if (line == "scan") {
    if (scanEndAt) { conPrint("[SCAN] Already running.\n"); return; }
    if (pClient && pClient->isConnected()) { pClient->disconnect(); delay(100); }
    NimBLEDevice::getScan()->stop(); delay(100);
    scanCount = 0;
    NimBLEDevice::getScan()->start(0, false);
    scanEndAt = millis() + 10000;
    conPrint("[SCAN] Scanning 10s — put keyboard into PAIRING MODE\n");
  } else if (line.startsWith("connect ")) {
    String mac = line.substring(8);
    mac.trim();
    if (mac.length() < 17) { conPrint("[ERR] Invalid MAC format.\n"); return; }
    if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; }
    reconnectAt = 0; reconnectFailures = 0;
    NimBLEDevice::deleteAllBonds();
    NimBLEAddress addr(mac.c_str(), 1);
    bool ok = false;
    for (int i = 0; i < CONNECT_TRIES && !ok; i++) {
      conPrintf("[BLE] Attempt %d/%d\n", i + 1, CONNECT_TRIES);
      ok = tryConnect(addr);
    }
    if (ok) {
      saveKeyboard(pClient->getPeerAddress());
      conPrintf("[NVS] Saved: %s\n", savedMAC);
    } else {
      conPrint("[BLE] Connection failed.\n");
    }
  } else if (line == "forget") {
    if (pClient && pClient->isConnected()) pClient->disconnect();
    prefs.begin(NVS_NS, false);
    prefs.clear();
    prefs.end();
    pLedChar = nullptr;
    pBatChar = nullptr;
    keepaliveAt = 0;
    NimBLEDevice::deleteAllBonds();
    memset(savedMAC, 0, sizeof(savedMAC));
    memset(prevKeys, 0, 6); prevMod = 0;
    reconnectAt = 0;
    conPrint("[NVS] Cleared. Use 'scan' then 'connect <mac>'.\n");
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
    g_cdcEnabled = enable;
    Serial.printf("[CDC] USB serial %s -- saved. Reboot to apply.\n",
                  enable ? "ENABLED" : "DISABLED");
  } else {
    conPrintf("[ERR] Unknown command: %s\n", line.c_str());
  }
}

// ── BLE daemon task ───────────────────────────────────────────────────────────
void bleDaemonTask(void* arg) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    if (strlen(savedMAC) == 0) continue;
    if (pClient && pClient->isConnected()) continue;
    if (reconnectAt != 0) continue;
    conPrint("[DAEMON] Keyboard lost, scheduling reconnect..." "\n");
    memset(prevKeys, 0, 6);
    prevMod        = 0;
    typematicKey   = 0;
    typematicArmed = false;
    typematicNext  = 0;
    hidKb.releaseAll();
    if (!reconnectScanActive) {
      reconnectScanActive = true;
      NimBLEDevice::getScan()->start(5000, false);
      conPrint("[SCAN] Waiting for keyboard to advertise..." "\n");
    }
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  // UART0 for debug (independent of USB HID port)
  Serial.begin(115200);
  delay(500);

  // Read NVS first so we know whether to register CDC in USB descriptor
  {
    Preferences p; p.begin(NVS_NS, true);
    g_cdcEnabled = p.getBool("cdc-en", false);
    p.end();
  }

  // USB HID + CDC composite init
  // CDC must be registered BEFORE USB.begin() — the descriptor is fixed at that point
  hidKb.begin();
  if (g_cdcEnabled) {
    USBSerial = new USBCDC();
    USBSerial->begin();
  }
  USB.begin();

  delay(1000);  // wait for USB enumeration

  Serial.println("\n================================");
  Serial.println("  BLE -> USB HID Bridge  v1.0");
  Serial.println("  ESP32-S3 N16R8");
  Serial.println("================================");

  // BLE
  NimBLEDevice::init(BRIDGE_NAME);
  NimBLEDevice::setPower(9);
  NimBLEDevice::setSecurityAuth(true, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::getScan()->setScanCallbacks(new MyScanCallbacks(), false);
  // Configure scan parameters once globally — applies to all scans including reconnect
  NimBLEDevice::getScan()->setActiveScan(true);
  NimBLEDevice::getScan()->setInterval(50);
  NimBLEDevice::getScan()->setWindow(45);

  xTaskCreatePinnedToCore(bleDaemonTask, "ble_daemon", 4096, nullptr, 1, nullptr, 0);

  loadSavedKeyboard();
  Serial.println("[NVS] ========== Stored configuration ==========");
  Serial.printf( "[NVS] Keyboard MAC:  %s\n", strlen(savedMAC) ? savedMAC : "(none)");
  Serial.println("[NVS] --- System ---");
  Serial.printf( "[NVS] CDC:           %s\n", g_cdcEnabled ? "enabled" : "disabled");
  Serial.println("[NVS] ============================================");
  if (strlen(savedMAC)) reconnectAt = millis() + 1500;
  if (g_cdcEnabled) Serial.println("[CDC] USB serial enabled.");
  else              Serial.println("[CDC] USB serial disabled. Use 'cdc on' to enable.");
  printHelp();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  handleSerial();

  // Scan end
  if (scanEndAt && millis() >= scanEndAt) {
    scanEndAt = 0;
    NimBLEDevice::getScan()->stop();
    if (scanCount == 0)
      conPrint("[SCAN] No devices found. Try again." "\n");
    else
      conPrintf("[SCAN] Done — %d devices. Use: connect <mac>\n", scanCount);
  }

  // Disconnection detection
  if (pClient && !pClient->isConnected() && strlen(savedMAC) > 0 && reconnectAt == 0) {
    conPrint("[BLE] Keyboard disconnected." "\n");
    NimBLEDevice::deleteClient(pClient); pClient = nullptr;
    memset(prevKeys, 0, 6); prevMod = 0;
    hidKb.releaseAll();
    reconnectFailures = 0;
    reconnectScanActive = true;
    NimBLEDevice::getScan()->start(5000, false);
    conPrint("[SCAN] Waiting for keyboard to advertise..." "\n");
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
        conPrintf("[BLE] Connect failed (%dx) — scanning again...\n", reconnectFailures);
        reconnectScanActive = true;
        NimBLEDevice::getScan()->start(5000, false);
      }
    }
  }

  // Keepalive — periodic battery read prevents keyboard from entering deep sleep
  if (keepaliveAt && millis() >= keepaliveAt && pBatChar &&
      pClient && pClient->isConnected()) {
    keepaliveAt = millis() + KEEPALIVE_MS;
    std::string val = pBatChar->readValue();
    if (!val.empty()) batteryLevel = (int)(uint8_t)val[0];
  }

  // Battery type-out (LCtrl+LAlt+PrtSc)
  if (batteryRequested) {
    batteryRequested = false;
    typematicKey = 0; typematicNext = 0;
    hidKb.releaseAll(); delay(50);
    typeBatteryViaUSB();
  }

  // Status type-out (LCtrl+LAlt+LShift+PrtSc)
  if (statusRequested) {
    statusRequested = false;
    typematicKey = 0; typematicNext = 0;
    hidKb.releaseAll(); delay(50);
    typeStatusViaUSB();
  }

  // Typematic repeat
  if (typematicKey && typematicNext && millis() >= typematicNext) {
    if (!typematicArmed) {
      typematicArmed = true;
      typematicNext  = millis() + TYPEMATIC_RATE_MS;
    } else {
      typematicNext = millis() + TYPEMATIC_RATE_MS;
    }
    // Re-assert modifiers for typematic repeat (prevMod already matches typematicMod)
    for (int bit = 0; bit < 8; bit++) {
      if (typematicMod & (1 << bit)) hidKb.pressRaw(0xE0 + bit);
    }
    hidKb.pressRaw(typematicKey);
  }

  delay(5);
}
