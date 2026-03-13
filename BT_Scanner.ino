/*
 * BLE Úplný skener — všechny dostupné informace
 * ESP32-WROOM-32 | NimBLE-Arduino 2.x | Arduino IDE 2.x
 *
 * NASTAVENÍ ARDUINO IDE:
 *   Board:  ESP32 Dev Module
 *   Port:   COM port ESP32
 */

#include <NimBLEDevice.h>

// Jak dlouho trvá jeden sken (sekundy)
#define SCAN_DURATION_SEC  8

// Pauza mezi skeny (sekundy)
#define SCAN_PAUSE_SEC     15

// ============================================================

static BLEUUID HID_SERVICE_UUID("00001812-0000-1000-8000-00805f9b34fb");
int scanCount = 0;

// Převod typu adresy na string
const char* addrTypeStr(uint8_t type) {
  switch (type) {
    case BLE_ADDR_PUBLIC:    return "Public";
    case BLE_ADDR_RANDOM:    return "Random";
    case BLE_ADDR_PUBLIC_ID: return "Public ID";
    case BLE_ADDR_RANDOM_ID: return "Random ID";
    default:                 return "Neznámý";
  }
}

// Převod appearance hodnoty na čitelný popis
const char* appearanceStr(uint16_t a) {
  switch (a) {
    case 0x0000: return "Neurčeno";
    case 0x0180: return "Obecné HID";
    case 0x0181: return "Klávesnice";
    case 0x0182: return "Myš";
    case 0x0183: return "Joystick";
    case 0x0184: return "Gamepad";
    case 0x03C0: return "Hodinky";
    case 0x0041: return "Sluchátka";
    case 0x0042: return "Hlasitý odposlech";
    case 0x0044: return "Reproduktor";
    default: {
      static char buf[16];
      snprintf(buf, sizeof(buf), "0x%04X", a);
      return buf;
    }
  }
}

// Pojmenuj UUID service — NimBLE 2.x: UUID16 přečteme přes toString() + strtoul
const char* serviceNameStr(const NimBLEUUID& uuid) {
  if (uuid.bitSize() != 16) return "";
  // toString() pro 16-bit UUID vrací "xxxx" (4 hex znaky)
  uint16_t uuid16 = (uint16_t)strtoul(uuid.toString().c_str(), nullptr, 16);
  switch (uuid16) {
    case 0x1800: return " (Generic Access)";
    case 0x1801: return " (Generic Attribute)";
    case 0x180A: return " (Device Information)";
    case 0x180F: return " (Battery Service)";
    case 0x1812: return " (HID - Human Interface Device)";
    case 0x1813: return " (Scan Parameters)";
    case 0x1819: return " (Location and Navigation)";
    case 0x181C: return " (User Data)";
    case 0x1824: return " (Transport Discovery)";
    default:     return "";
  }
}

// Vypiš všechny UUID services
void printServices(const NimBLEAdvertisedDevice* dev) {
  if (dev->haveServiceUUID()) {
    Serial.printf("      Services (%d):\n", dev->getServiceUUIDCount());
    for (int i = 0; i < (int)dev->getServiceUUIDCount(); i++) {
      NimBLEUUID uuid = dev->getServiceUUID(i);
      Serial.printf("        [%d] %s%s\n",
                    i,
                    uuid.toString().c_str(),
                    serviceNameStr(uuid));
    }
  } else {
    Serial.println("      Services: (žádné v advertisementu)");
  }
}

// Vypiš Service Data
void printServiceData(const NimBLEAdvertisedDevice* dev) {
  if (dev->haveServiceData()) {
    Serial.printf("      Service Data (%d):\n", dev->getServiceDataCount());
    for (int i = 0; i < (int)dev->getServiceDataCount(); i++) {
      Serial.printf("        [%d] UUID: %s\n", i,
                    dev->getServiceDataUUID(i).toString().c_str());
      std::string data = dev->getServiceData(i);
      Serial.print("             Data (hex): ");
      for (size_t j = 0; j < data.size(); j++) {
        Serial.printf("%02X ", (uint8_t)data[j]);
      }
      Serial.println();
    }
  }
}

// Vypiš Manufacturer Data
void printManufacturerData(const NimBLEAdvertisedDevice* dev) {
  if (dev->haveManufacturerData()) {
    std::string mfr = dev->getManufacturerData();
    Serial.print("      Manufacturer Data (hex): ");
    for (size_t i = 0; i < mfr.size(); i++) {
      Serial.printf("%02X ", (uint8_t)mfr[i]);
    }
    // První dva bajty jsou Company ID (little-endian)
    if (mfr.size() >= 2) {
      uint16_t companyId = ((uint8_t)mfr[1] << 8) | (uint8_t)mfr[0];
      Serial.printf("\n      Company ID: 0x%04X", companyId);
      switch (companyId) {
        case 0x004C: Serial.print(" (Apple)"); break;
        case 0x0006: Serial.print(" (Microsoft)"); break;
        case 0x00E0: Serial.print(" (Google)"); break;
        case 0x0059: Serial.print(" (Nordic Semiconductor)"); break;
        case 0x0499: Serial.print(" (Ruuvi)"); break;
        case 0x05A7: Serial.print(" (Sonos)"); break;
        case 0x008A: Serial.print(" (Realtek)"); break;
        default: break;
      }
    }
    Serial.println();
  }
}

// Hlavní funkce výpisu jednoho zařízení
void printDevice(int index, const NimBLEAdvertisedDevice* dev) {
  bool isHID = dev->haveServiceUUID() &&
               dev->isAdvertisingService(HID_SERVICE_UUID);

  Serial.println("  ----------------------------------------");
  Serial.printf("  Zařízení #%d %s\n", index,
                isHID ? "*** BLE HID KLÁVESNICE ***" : "");
  Serial.println("  ----------------------------------------");

  // Základní identifikace
  Serial.printf("      MAC adresa:   %s\n",
                dev->getAddress().toString().c_str());
  Serial.printf("      Typ adresy:   %s\n",
                addrTypeStr(dev->getAddress().getType()));
  Serial.printf("      RSSI:         %d dBm\n", dev->getRSSI());

  // Název
  if (dev->haveName()) {
    Serial.printf("      Název:        %s\n", dev->getName().c_str());
  } else {
    Serial.println("      Název:        (neuvedeno)");
  }

  // Appearance
  if (dev->haveAppearance()) {
    Serial.printf("      Appearance:   %s\n",
                  appearanceStr(dev->getAppearance()));
  }

  // TX Power
  if (dev->haveTXPower()) {
    Serial.printf("      TX Power:     %d dBm\n", dev->getTXPower());
  }

  // Typ advertisementu — opravená konstanta pro NimBLE 2.x
  Serial.print("      Adv. typ:     ");
  switch (dev->getAdvType()) {
    case BLE_HCI_ADV_TYPE_ADV_IND:
      Serial.println("ADV_IND (connectable, scannable)"); break;
    case BLE_HCI_ADV_TYPE_ADV_DIRECT_IND_HD:
      Serial.println("ADV_DIRECT_IND HD (directed)"); break;
    case BLE_HCI_ADV_TYPE_ADV_SCAN_IND:
      Serial.println("ADV_SCAN_IND (scannable, non-connectable)"); break;
    case BLE_HCI_ADV_TYPE_ADV_NONCONN_IND:
      Serial.println("ADV_NONCONN_IND (non-connectable)"); break;
    case BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP:   // ← opravená konstanta
      Serial.println("SCAN_RSP (scan response)"); break;
    default:
      Serial.printf("0x%02X\n", dev->getAdvType()); break;
  }

  // Connectable
  Serial.printf("      Připojitelné: %s\n",
                dev->isConnectable() ? "ANO" : "NE");

  // Services
  printServices(dev);

  // Service Data
  printServiceData(dev);

  // Manufacturer Data
  printManufacturerData(dev);

  // URI
  if (dev->haveURI()) {
    Serial.printf("      URI:          %s\n", dev->getURI().c_str());
  }

  Serial.println();
}

// ============================================================

void doScan() {
  scanCount++;
  Serial.println("\n========================================");
  Serial.printf(" Sken #%d (trvání %ds)\n", scanCount, SCAN_DURATION_SEC);
  Serial.println("========================================\n");

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->clearResults();

  NimBLEScanResults results = scan->getResults(SCAN_DURATION_SEC * 1000, false);

  int total    = results.getCount();
  int hidCount = 0;

  Serial.printf("Nalezeno %d BLE zařízení:\n\n", total);

  for (int i = 0; i < total; i++) {
    const NimBLEAdvertisedDevice* dev = results.getDevice(i);
    printDevice(i + 1, dev);
    if (dev->haveServiceUUID() &&
        dev->isAdvertisingService(HID_SERVICE_UUID)) {
      hidCount++;
    }
  }

  Serial.println("========================================");
  Serial.printf(" Souhrn: celkem %d zařízení, z toho %d HID\n",
                total, hidCount);
  Serial.printf(" Další sken za %d sekund...\n", SCAN_PAUSE_SEC);
  Serial.println("========================================\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("================================");
  Serial.println("  BLE Úplný skener");
  Serial.println("  ESP32-WROOM-32 | NimBLE 2.x");
  Serial.printf ("  Sken každých %ds, trvání %ds\n",
                 SCAN_PAUSE_SEC, SCAN_DURATION_SEC);
  Serial.println("================================");

  NimBLEDevice::init("ESP32-Scanner");
  NimBLEDevice::setPower(9);

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);  // active = dostane Scan Response (název, TX power)
  scan->setInterval(100);
  scan->setWindow(60);

  doScan();
}

void loop() {
  delay(SCAN_PAUSE_SEC * 1000);
  doScan();
}
