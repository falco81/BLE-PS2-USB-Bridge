/*
 * BLE → USB HID Keyboard Bridge  (Varianta 2 — ESP32-S3)
 * ========================================================
 * Hardware : ESP32-S3 N16R8
 * Framework: Arduino ESP32 core 3.x, NimBLE-Arduino 2.x
 * IDE      : Arduino IDE 2.x
 *
 * Funkce:
 *   - Připojí se k BLE HID klávesnici jako host
 *   - Přeposílá stisknuté klávesy jako USB HID klávesnice
 *   - Nativní USB ESP32-S3 (žádný CH340 / převodník)
 *   - Typematic (opakování kláves při držení)
 *   - Auto-reconnect po odpojení / restartu klávesnice
 *   - NVS — spárovaná klávesnice pamatována po restartu
 *   - Serial konzole pro scan / connect / forget / status
 *
 * NASTAVENÍ ARDUINO IDE:
 *   Board  : ESP32S3 Dev Module
 *   USB Mode: USB-OTG (TinyUSB)   ← NUTNÉ
 *   Upload : UART0 / USB CDC
 *   Port   : COM port ESP32-S3
 *
 * Zapojení:
 *   USB-C kabel → PC (USB HID výstup + napájení)
 *   Žádný level shifter, žádné extra součástky.
 */

// ── USB HID (nativní ESP32-S3) ────────────────────────────────────────────────
#include "USB.h"
#include "USBHIDKeyboard.h"

// ── BLE ───────────────────────────────────────────────────────────────────────
#include <NimBLEDevice.h>
#include <Preferences.h>

// ── Konfigurace ───────────────────────────────────────────────────────────────
#define BRIDGE_NAME          "BLE-USB-Bridge"
#define NVS_NS               "ble-usb"
#define NVS_KEY_MAC          "mac"
#define NVS_KEY_TYPE         "type"
#define CONNECT_TRIES        3

// Typematic — stejné hodnoty jako PS/2 varianta
#define TYPEMATIC_DELAY_MS   500   // ms do začátku opakování
#define TYPEMATIC_RATE_MS     50   // ms mezi opakováními (20 kláves/s)

// ── Globální objekty ──────────────────────────────────────────────────────────
static USBHIDKeyboard    hidKb;

static NimBLEClient*     pClient           = nullptr;
static unsigned long     reconnectAt       = 0;
static int               reconnectFailures = 0;

static Preferences       prefs;
static char              savedMAC[18]      = "";
static uint8_t           savedType         = 1;

static uint8_t           prevKeys[6]       = {0};
static uint8_t           prevMod           = 0;

static BLEUUID HID_SERVICE_UUID("00001812-0000-1000-8000-00805f9b34fb");
static BLEUUID HID_REPORT_UUID ("00002a4d-0000-1000-8000-00805f9b34fb");

static unsigned long     scanEndAt  = 0;
static int               scanCount  = 0;

// Typematic stav
static unsigned long     typematicNext  = 0;
static bool              typematicArmed = false;
static uint8_t           typematicKey   = 0;   // HID keycode držené klávesy
static uint8_t           typematicMod   = 0;   // modifikátory při držení

// ── USB HID — odesílání kláves ────────────────────────────────────────────────
//
// BLE HID a USB HID používají identické keykódy (HID Usage Table).
// Stačí přeposlat report přímo.
//
// USB HID modifier byte:
//   bit 0 = LCtrl  bit 1 = LShift  bit 2 = LAlt  bit 3 = LGUI
//   bit 4 = RCtrl  bit 5 = RShift  bit 6 = RAlt  bit 7 = RGUI

void usbApplyModifiers(uint8_t mod) {
  // Uvolni všechny modifikátory pak nastav aktuální stav
  // USBHIDKeyboard::pressModifier() / releaseModifier() pracují s maskou
  hidKb.releaseModifier(0xFF);   // uvolní všechny
  if (mod) hidKb.pressModifier(mod);
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
  if (len < 3) return;
  uint8_t  mod  = data[0];
  uint8_t* keys = data + 2;

  Serial.print("[HID] ");
  for (size_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.println();

  // Modifikátory — nastav celou masku najednou
  if (mod != prevMod) {
    usbApplyModifiers(mod);
    // Log
    for (int bit = 0; bit < 8; bit++) {
      uint8_t mask = 1 << bit;
      bool was = (prevMod & mask) != 0;
      bool is  = (mod     & mask) != 0;
      if (is  && !was) Serial.printf("[+] %s\n", modName(bit));
      if (!is &&  was) Serial.printf("[-] %s\n", modName(bit));
    }
  }

  // Uvolněné klávesy
  for (int i = 0; i < 6; i++) {
    if (prevKeys[i] == 0 || prevKeys[i] == 0x01) continue;
    bool found = false;
    for (int j = 0; j < 6; j++) if (keys[j] == prevKeys[i]) { found = true; break; }
    if (!found) {
      Serial.printf("[-] %s\n", hidKeyName(prevKeys[i]));
      usbKeyUp(prevKeys[i]);
    }
  }

  // Stisknuté klávesy
  for (int i = 0; i < 6; i++) {
    if (keys[i] == 0 || keys[i] == 0x01) continue;
    bool found = false;
    for (int j = 0; j < 6; j++) if (prevKeys[j] == keys[i]) { found = true; break; }
    if (!found) {
      Serial.printf("[+] %s\n", hidKeyName(keys[i]));
      usbKeyDown(keys[i]);
    }
  }

  prevMod = mod;
  memcpy(prevKeys, keys, 6);

  // Typematic — první držená klávesa (bez CapsLock 0x39)
  typematicKey = 0;
  typematicMod = mod;
  for (int i = 0; i < 6; i++) {
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
    if (!dev->haveName() || dev->getName().length() == 0) return;
    Serial.printf("  %-30s  %s\n",
      dev->getName().c_str(), dev->getAddress().toString().c_str());
    scanCount++;
  }
  void onScanEnd(const NimBLEScanResults& r, int reason) override {}
};

// ── BLE připojení ─────────────────────────────────────────────────────────────
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

  Serial.printf("[BLE] Pripojuji se k %s ...\n", addr.toString().c_str());
  pClient = NimBLEDevice::createClient();
  pClient->setConnectionParams(6, 12, 0, 3200);
  pClient->setConnectTimeout(12);

  if (!pClient->connect(addr)) {
    NimBLEDevice::deleteClient(pClient); pClient = nullptr;
    Serial.println("[BLE] Pripojeni selhalo.");
    return false;
  }

  if (pClient->secureConnection()) Serial.println("[BLE] Bonding OK");

  NimBLERemoteService* svc = pClient->getService(HID_SERVICE_UUID);
  if (!svc) {
    Serial.println("[BLE] HID service nenalezena.");
    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient); pClient = nullptr;
    return false;
  }

  int subs = 0;
  const std::vector<NimBLERemoteCharacteristic*>& chars = svc->getCharacteristics(true);
  for (NimBLERemoteCharacteristic* c : chars) {
    if (c->getUUID() == HID_REPORT_UUID && c->canNotify()) {
      c->subscribe(true, notifyCallback);
      subs++;
    }
  }

  Serial.printf("[BLE] Pripojeno. Subscriptions: %d\n", subs);
  return subs > 0;
}

// ── Serial konzole ────────────────────────────────────────────────────────────
void printHelp() {
  Serial.println("\n--- Prikazy ---");
  Serial.println("  scan          BLE sken (10s)");
  Serial.println("  connect <mac> Pripojit a ulozit");
  Serial.println("  forget        Smazat parovanou klavesnici");
  Serial.println("  status        Stav pripojeni");
  Serial.println("  help          Tento vypis");
  Serial.println("---------------");
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line == "help") {
    printHelp();
  } else if (line == "scan") {
    if (scanEndAt) { Serial.println("[SCAN] Jiz probiha."); return; }
    if (pClient && pClient->isConnected()) { pClient->disconnect(); delay(100); }
    scanCount = 0;
    NimBLEDevice::getScan()->start(10000, false);
    scanEndAt = millis() + 10000;
    Serial.println("[SCAN] Sken 10s...");
  } else if (line.startsWith("connect ")) {
    String mac = line.substring(8);
    mac.trim();
    if (mac.length() < 17) { Serial.println("[ERR] Spatny format MAC."); return; }
    if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; }
    NimBLEAddress addr(mac.c_str(), 1);
    bool ok = false;
    for (int i = 0; i < CONNECT_TRIES && !ok; i++) {
      Serial.printf("[BLE] Pokus %d/%d\n", i + 1, CONNECT_TRIES);
      ok = tryConnect(addr);
    }
    if (ok) {
      saveKeyboard(addr);
      Serial.printf("[NVS] Ulozeno: %s\n", savedMAC);
    } else {
      Serial.println("[BLE] Pripojeni se nezdarilo.");
    }
  } else if (line == "forget") {
    if (pClient && pClient->isConnected()) pClient->disconnect();
    prefs.begin(NVS_NS, false);
    prefs.clear();
    prefs.end();
    NimBLEDevice::deleteAllBonds();
    memset(savedMAC, 0, sizeof(savedMAC));
    memset(prevKeys, 0, 6); prevMod = 0;
    reconnectAt = 0;
    Serial.println("[NVS] Smazano. Pouzij 'scan' a 'connect <mac>'.");
  } else if (line == "status") {
    Serial.printf("[STATUS] BLE: %s\n",
      (pClient && pClient->isConnected()) ? "CONNECTED" : "DISCONNECTED");
    Serial.printf("[STATUS] Ulozena MAC: %s\n",
      strlen(savedMAC) ? savedMAC : "(zadna)");
    Serial.printf("[STATUS] Bondy: %d\n", NimBLEDevice::getNumBonds());
  } else {
    Serial.printf("[ERR] Neznamy prikaz: %s\n", line.c_str());
  }
}

// ── BLE daemon task ───────────────────────────────────────────────────────────
void bleDaemonTask(void* arg) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    if (strlen(savedMAC) == 0) continue;
    if (pClient && pClient->isConnected()) continue;
    if (reconnectAt != 0) continue;
    Serial.println("[DAEMON] Klavesnice ztracena, planuji reconnect...");
    memset(prevKeys, 0, 6);
    prevMod        = 0;
    typematicKey   = 0;
    typematicArmed = false;
    typematicNext  = 0;
    hidKb.releaseAll();   // uvolni všechny USB klávesy
    reconnectAt = millis() + 500;
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  // UART0 pro debug (nezávislé na USB HID)
  Serial.begin(115200);
  delay(500);

  // USB HID init
  hidKb.begin();
  USB.begin();

  delay(1000);  // počkej na USB enumerate

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

  xTaskCreatePinnedToCore(bleDaemonTask, "ble_daemon", 4096, nullptr, 1, nullptr, 0);

  if (loadSavedKeyboard()) {
    Serial.printf("[NVS] Ulozena klavesnice: %s\n", savedMAC);
    reconnectAt = millis() + 1500;  // trochu delší čekání — USB potřebuje čas
  } else {
    Serial.println("[NVS] Zadna klavesnice. Pouzij 'scan' pak 'connect <mac>'.");
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
      Serial.println("[SCAN] Zadne zarizeni. Zkus znovu.");
    else
      Serial.printf("[SCAN] Hotovo — %d zarizeni. Pouzij: connect <mac>\n", scanCount);
  }

  // Detekce odpojení
  if (pClient && !pClient->isConnected() && strlen(savedMAC) > 0 && reconnectAt == 0) {
    Serial.println("[BLE] Klavesnice odpojena.");
    NimBLEDevice::deleteClient(pClient); pClient = nullptr;
    memset(prevKeys, 0, 6); prevMod = 0;
    hidKb.releaseAll();
    reconnectFailures = 0;
    reconnectAt = millis() + 2000;
  }

  // Reconnect
  if (reconnectAt && millis() >= reconnectAt && strlen(savedMAC) > 0) {
    reconnectAt = 0;
    if (!pClient || !pClient->isConnected()) {
      if (scanEndAt) { NimBLEDevice::getScan()->stop(); scanEndAt = 0; }
      NimBLEAddress addr(savedMAC, savedType);
      if (tryConnect(addr)) {
        reconnectFailures = 0;
      } else {
        reconnectFailures++;
        Serial.printf("[BLE] Reconnect selhal (%dx). Dalsi pokus za 2s\n", reconnectFailures);
        reconnectAt = millis() + 2000;
      }
    }
  }

  // Typematic opakování
  if (typematicKey && typematicNext && millis() >= typematicNext) {
    if (!typematicArmed) {
      typematicArmed = true;
      typematicNext  = millis() + TYPEMATIC_RATE_MS;
    } else {
      typematicNext = millis() + TYPEMATIC_RATE_MS;
    }
    usbApplyModifiers(typematicMod);
    hidKb.pressRaw(typematicKey);
  }

  delay(5);
}
