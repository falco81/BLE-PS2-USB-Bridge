# BLE → PS/2 AT Keyboard Bridge

> ESP32 firmware that acts as a **PS/2 AT keyboard emulator** — connects to any BLE HID keyboard and forwards keystrokes to a PC via the PS/2 port.

Ideal for retro PCs, vintage hardware, industrial machines or any device with an AT DIN-5 or PS/2 keyboard port that you want to use with a modern wireless keyboard.

---

## Block Diagram

![Block diagram](doc/block_diagram.svg)

---

## Features

- **BLE HID host** — connects to any Bluetooth LE keyboard (NimBLE-Arduino 2.x)
- **PS/2 AT emulator** — full scan code set 2, open-drain bus, FreeRTOS tasks
- **Typematic** — key repeat with 500 ms initial delay, 20 keys/s rate
- **Scan-before-connect** — waits for keyboard to start advertising before attempting connection, eliminates blind retry loops
- **Auto-reconnect** — BLE daemon task detects disconnection and reconnects automatically
- **Battery level** — reads and displays keyboard battery percentage if supported
- **BIOS compatible** — responds to all POST commands including `GET_DEVICE_ID` (0xF2 → `0xAB 0x83`)
- **Serial console** — `scan` / `connect` / `forget` / `status` commands at 115200 baud
- **LED sync** — NumLock / CapsLock / ScrollLock state forwarded back to the BLE keyboard so indicators stay in sync
- **Battery shortcut** — press **LCtrl+LAlt+PrintScreen** to type the keyboard battery percentage (e.g. `78%`)
- **Status shortcut** — press **LCtrl+LAlt+LShift+PrintScreen** to type the full bridge status into any text field
- **NVS storage** — paired keyboard remembered across reboots
- **Keepalive** — periodic GATT read every 2 s maintains BLE connection and updates battery level. Note: on some keyboards the key scanner MCU has independent sleep logic and may still show input latency after long idle periods — this is a keyboard firmware limitation
- **WiFi disabled** — WiFi stack is deinitialised at startup, saving ~20 mA

---

## Hardware

### Components

| Component | Description |
|-----------|-------------|
| ESP32-WROOM-32 (MH-ET LIVE MiniKit) | Main microcontroller |
| BSS138 bidirectional level shifter | 3.3 V ↔ 5 V logic translation |
| AT DIN-5 male connector | Keyboard port on PC motherboard |

### Connectors

#### AT DIN-5 (used in this project)

<img src="doc/din5_male.png" width="200" alt="AT DIN-5 male connector"/>

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | CLK | Keyboard clock |
| 2 | DATA | Keyboard data |
| 3 | RESET | Not used |
| 4 | GND | Ground |
| 5 | +5V | Power |

#### PS/2 Mini-DIN 6 (alternative — note different pinout!)

<img src="doc/minidins6_male.png" width="200" alt="PS/2 Mini-DIN 6 male connector"/>

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | DATA | Keyboard data |
| 2 | — | Not used |
| 3 | GND | Ground |
| 4 | +5V | Power |
| 5 | CLK | Keyboard clock |
| 6 | — | Not used |

> ⚠️ **Note:** CLK and DATA are **swapped** between DIN-5 and Mini-DIN 6. If switching connector types, swap GPIO18 and GPIO19 in the code or rewire at the level shifter.

### Level Shifter

<img src="doc/level_shifter.png" width="220" alt="BSS138 IIC I2C 5V to 3.3V bidirectional level shifter"/>

The BSS138-based bidirectional level shifter translates between ESP32's 3.3 V GPIO and the PC's 5 V PS/2 bus. It has built-in pull-up resistors on both sides — **no external resistors needed**. The LV GND and HV GND are connected internally on the module.

The module has two sides:
- **LV side** — connect to ESP32 (3.3 V logic)
- **HV side** — connect to PS/2 port (5 V logic)

---

### Wiring Diagram

![Wiring diagram](doc/wiring_diagram.svg)

#### Step-by-step connections

| ESP32 pin | Level shifter | DIN-5 pin | Signal |
|-----------|--------------|-----------|--------|
| 3V3 | LV | — | 3.3 V reference for LV side |
| VIN (5V) | HV | pin 5 | 5 V reference for HV side + power |
| GND | GND (both) | pin 4 | Common ground |
| GPIO19 | A1 → B1 | pin 1 | CLK |
| GPIO18 | A2 → B2 | pin 2 | DATA |
| — | — | pin 3 | Not connected |

> 💡 **Why a level shifter?** PS/2 is a 5 V open-collector bus. ESP32 GPIO is 3.3 V tolerant max. A simple voltage divider pulls idle CLK/DATA lines too low, causing the PC to see them as permanently asserted (inhibit state). The BSS138 level shifter correctly translates logic levels while maintaining open-drain behaviour.

> 💡 **Open-drain GPIO:** The firmware configures CLK and DATA as `GPIO_MODE_OUTPUT_OD` (open-drain). Driving LOW pulls the line down, releasing to INPUT allows the pull-up to float it HIGH — exactly as required by the PS/2 protocol.

---

## Software

### Requirements

| Tool | Version |
|------|---------|
| Arduino IDE | 2.x |
| ESP32 Arduino core | ≥ 3.x |
| NimBLE-Arduino | 2.x |

### Installation

1. Install **ESP32 Arduino core** via Boards Manager
2. Install **NimBLE-Arduino** via Library Manager
3. Open `ble_ps2_bridge.ino` in Arduino IDE
4. Select board: **ESP32 Dev Module** (or **MH ET Live ESP32MiniKit**)
5. Upload

### Configuration

At the top of the sketch:

```cpp
#define PS2_CLK_PIN   19    // CLK GPIO
#define PS2_DAT_PIN   18    // DATA GPIO

#define TYPEMATIC_DELAY_MS  500   // ms before key repeat starts
#define TYPEMATIC_RATE_MS    50   // ms between repeats (50 = 20 keys/s)
```

---

## Usage

Connect via **Serial monitor at 115200 baud**.

### First-time pairing

```
scan                        # scan BLE devices for 10 seconds
                            # put keyboard in pairing mode first
  MyKeyboard  aa:bb:cc:dd:ee:ff

connect aa:bb:cc:dd:ee:ff   # connect and save
```

### Commands

| Command | Description |
|---------|-------------|
| `scan` | Scan BLE 10 s, show named devices |
| `connect <mac>` | Connect to MAC address and save (3 attempts) |
| `forget` | Clear saved keyboard and all bonds |
| `status` | Show full connection info, NVS data, battery level |
| `help` | Show command list |

### Status output

The `status` command and **Ctrl+Alt+PrintScreen** both produce:

```
--- BLE-PS2 Bridge status ---
BLE:      CONNECTED
Peer MAC: dc:35:77:e9:cf:a1
RSSI:     -52 dBm
Interval: 48 ms
Encrypt:  yes
Bonded:   yes
Battery:  78%

NVS MAC:  dc:35:77:e9:cf:a1
Bonds:    1
-----------------------------
```

Battery is shown only if the keyboard supports the BLE Battery Service (UUID 0x180F). If not supported, `Battery: unknown` is displayed.

### Keyboard shortcuts

| Combination | Output |
|-------------|--------|
| **LCtrl + LAlt + PrtSc** | Battery percentage |
| **LCtrl + LAlt + LShift + PrtSc** | Full bridge status block |

Both shortcuts type into whatever text field has focus — Notepad, terminal, browser address bar, etc. They work without access to the Serial monitor.

Only **Left** modifier keys are recognised. The bridge releases all held modifier keys and stops typematic before typing the output, so the text is not corrupted by Ctrl/Alt being held on the PC side.

**LCtrl + LAlt + PrtSc** example output:
```
78%
```
If the keyboard does not support battery reporting: `?%`

**LCtrl + LAlt + LShift + PrtSc** example output:
```

--- BLE-PS2 Bridge status ---
BLE:      CONNECTED
Peer MAC: dc:35:77:e9:cf:a1
RSSI:     -52 dBm
Interval: 48 ms
Encrypt:  yes
Bonded:   yes
Battery:  78%

NVS MAC:  dc:35:77:e9:cf:a1
Bonds:    1
-----------------------------
```
When disconnected:
```

--- BLE-PS2 Bridge status ---
BLE:      DISCONNECTED
Scanning for keyboard...
Failures: 3

NVS MAC:  dc:35:77:e9:cf:a1
Bonds:    1
-----------------------------
```

### Lock key LED synchronisation

NumLock, CapsLock and ScrollLock indicator LEDs on the BLE keyboard stay in sync with the PC state automatically.

In the **PS/2 variant**, the PC sends a `0xED` LED command to the bridge whenever lock state changes. The bridge remaps the bitmask (PS/2 and BLE HID use different bit order) and writes it to the BLE keyboard's HID Output Report characteristic.

In the **USB variant**, the PC sends a USB HID Output Report directly. Since Arduino core does not expose this callback, the bridge tracks lock key toggle events locally and updates the BLE keyboard accordingly.

| PS/2 bit | BLE HID bit | Key |
|----------|------------|-----|
| bit 1 | bit 0 | NumLock |
| bit 2 | bit 1 | CapsLock |
| bit 0 | bit 2 | ScrollLock |

### Automatic reconnect

When the keyboard disconnects, the firmware starts a BLE scan and waits for the keyboard to begin advertising, then connects immediately. This eliminates the repeated 12-second connection timeouts that occur when connecting to a device that has not yet started advertising.

---

## PS/2 Protocol Implementation

### Startup sequence

On boot the firmware immediately sends **BAT (Basic Assurance Test) `0xAA`** to the PC before BLE initialises. BIOS waits up to ~500 ms for this byte.

### Host command handling

| Command | Response | Notes |
|---------|----------|-------|
| `0xFF` RESET | `0xFA` + `0xAA` | Re-enables data reporting |
| `0xF2` GET_DEVICE_ID | `0xFA` + `0xAB` + `0x83` | **Critical** — BIOS identifies keyboard type |
| `0xED` SET LEDs | `0xFA` + read byte + `0xFA` | NumLock/CapsLock/ScrollLock |
| `0xF3` Typematic rate | `0xFA` + read byte + `0xFA` | Accepted, not applied |
| `0xF0` Scan code set | `0xFA` + read byte + `0xFA` | Always set 2 |
| `0xF4` Enable reporting | `0xFA` | |
| `0xF5` Disable reporting | `0xFA` | |
| `0xEE` Echo | `0xEE` | |
| `0xFE` Resend | `0xFA` | |
| default | `0xFA` | |

### FreeRTOS architecture

```
Core 0                          Core 1 (not used for PS/2)
──────────────────────          ──────────────────────────
BLE stack (NimBLE)
  └─ notifyCallback()
       └─ keyboard.keyHid_send()
            └─ xQueueSend()
                                ps2host task  (priority 10)
                                  checks CLK/DATA every 9 ms
                                  → reply_to_host()

                                ps2send task  (priority 9)
                                  xQueueReceive() → ps2_write()

bleDaemonTask (priority 1)
  checks connection every 3 s
  triggers scan-before-connect if lost
```

### Timing constants

| Constant | Value | Source |
|----------|-------|--------|
| CLK half period | 40 µs | PS/2 spec (30–50 µs) |
| Byte interval | 500 µs | Measured from reference hardware |
| Host poll interval | 9 ms | < 10 ms per spec |
| Supervision timeout | 32 s | BLE connection parameter |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| *Keyboard error or not present* | BAT not received in time | Check level shifter wiring, 5V present on HV side |
| BIOS OK but OS fails | Supervision timeout during POST | Already fixed — supervision = 32 s |
| Keys not repeating | — | Typematic implemented in firmware |
| Wrong keys | Wrong pin assignment | DIN-5: pin1=CLK, pin2=DATA. Mini-DIN 6: pin5=CLK, pin1=DATA |
| Can't connect BLE | Keyboard not in pairing mode | Use `forget` then `scan`, put keyboard into pairing mode |
| Connects but no input | Wrong HID report format | Check Serial log — `[HID]` lines should appear on keypress |
| Reconnect takes a while | Keyboard not advertising yet | Normal — firmware scans and waits, connects as soon as keyboard is visible |
| Lock key LEDs wrong | Bridge just connected | Press the lock key once to resync state |
| 5–10 s input delay after idle | Keyboard key scanner MCU entered deep sleep | This is a keyboard firmware limitation — ESP32 cannot wake the key scanner, only the BLE radio. Try a different keyboard or check if a firmware update is available. |

---

## Credits & References

- PS/2 protocol implementation based on [esp32-bt2ps2](https://github.com/hamberthm/esp32-bt2ps2) by Humberto Mockel
- PS/2 scan codes: [computer-engineering.org](http://www.computer-engineering.org/ps2keyboard/scancodes2.html)
- AT/PS2 connector pinout: [5kw.dk](https://5kw.dk/Hobby/electronics/connectors/at_keyboard_connector.htm)

---

## License

MIT

---

---

# Variant 2 — BLE → USB HID (ESP32-S3 N16R8)

BLE keyboard → **USB HID keyboard** using the native USB interface of the ESP32-S3. No level shifter, no PS/2 cable — a single USB-C cable both powers the module and carries HID data.

## Hardware

### ESP32-S3 N16R8

<img src="doc/esp32s3_module.png" width="320" alt="ESP32-S3 N16R8 development module"/>

| Parameter | Value |
|-----------|-------|
| SoC | ESP32-S3 |
| Flash | 16 MB |
| PSRAM | 8 MB (Octal) |
| BLE | 5.0 (NimBLE) |
| WiFi | 802.11 b/g/n |
| USB | Native USB OTG (no CH340) |
| Dimensions | 51 × 25 mm |

> The ESP32-S3 has **native USB** built into the chip — it can present itself as a USB HID device without any converter. BLE HID keycodes (Usage Page 0x07) are identical to USB HID keycodes, so forwarding is direct with no translation table.

### Wiring

```
BLE Keyboard  ──(BLE 5.0)──►  ESP32-S3  ──(USB-C)──►  PC
```

**No extra components.** A single USB-C cable provides both power and HID output to the PC.

### USB ports on the ESP32-S3 devkit

The devkit has two USB-C ports:

| Port | Chip | Use |
|------|------|-----|
| UART | CH340 / CP2102 | Upload firmware, Serial Monitor debug |
| USB-OTG | Native ESP32-S3 | HID output to PC (keyboard) |

Both can be connected simultaneously. Use the UART port for Arduino IDE uploads and the USB-OTG port for the target PC.

## Block Diagram

![Block diagram S3](doc/block_diagram_s3.svg)

## Software — `ble_usb_bridge_s3.ino`

### Arduino IDE Settings

| Setting | Value |
|---------|-------|
| Board | **ESP32S3 Dev Module** |
| USB Mode | **USB-OTG (TinyUSB)** ← required! |
| USB CDC On Boot | Disabled |
| Flash Size | 16MB |
| PSRAM | OPI PSRAM |
| Upload Mode | UART0 / USB-CDC |

> ⚠️ **USB Mode must be set to `USB-OTG (TinyUSB)`** — without this USB HID will not work. Set it in the `Tools` menu in Arduino IDE.

### Required Libraries

| Library | Source |
|---------|--------|
| NimBLE-Arduino 2.x | Library Manager |
| USB.h + USBHIDKeyboard.h | included in ESP32 Arduino core 3.x |

### How It Works

```
notifyCallback()          ← BLE HID notify from keyboard
  └─ processHIDReport()
       ├─ usbApplyModifiers(mod)   → hidKb.pressRaw() / releaseRaw() per bit
       ├─ usbKeyUp(keycode)        → hidKb.releaseRaw(keycode)
       └─ usbKeyDown(keycode)      → hidKb.pressRaw(keycode)

loop()
  └─ Typematic: hidKb.pressRaw(typematicKey) every 50 ms after 500 ms hold
  └─ statusRequested: typeStatusViaUSB() on Ctrl+Alt+PrintScreen

bleDaemonTask (FreeRTOS, core 0)
  └─ check every 3 s → hidKb.releaseAll() + scan-before-connect
```

Since the BLE HID Usage Table (Keyboard/Keypad, Usage Page 0x07) is identical to USB HID, keycodes are forwarded directly without a translation table.

### Comparison with PS/2 Variant

| | Variant 1 (PS/2) | Variant 2 (USB) |
|--|------------------|-----------------|
| Hardware | ESP32-WROOM-32 + level shifter | ESP32-S3 |
| Output | AT DIN-5 / PS/2 | USB-C HID |
| Extra components | BSS138 level shifter | None |
| Power | 5V from PS/2 or USB | USB-C cable |
| Compatibility | Retro PC, industrial | Modern PC, Mac, Linux |
| BIOS POST | ✅ (BAT 0xAA, GET_DEVICE_ID) | ✅ (USB enumeration) |
| Key codes | PS/2 Set 2 | USB HID Usage |
| Driver | None (PS/2 native) | None (HID plug & play) |
| WiFi disabled | ✅ saves ~20 mA | — |
| Battery level | ✅ | ✅ |
| LCtrl+LAlt+PrtSc → battery % | ✅ | ✅ |
| LCtrl+LAlt+LShift+PrtSc → status | ✅ via PS/2 | ✅ via USB HID |
| LED sync | ✅ via 0xED forward | ✅ via toggle tracking |

### Serial Console

Debug output goes through **UART0** (UART port on the devkit) — independent of the USB HID port. Commands are the same as in the PS/2 variant: `scan` / `connect <mac>` / `forget` / `status` / `help`.

### Configuration

```cpp
#define TYPEMATIC_DELAY_MS   500   // ms before repeat starts
#define TYPEMATIC_RATE_MS     50   // ms between repeats (20 keys/s)
```

---

---

# Tools — BT Scanner

`BT_Scanner.ino` scans nearby BLE devices. Use it to find the MAC address of your keyboard before pairing.

## Usage

1. Upload `BT_Scanner.ino` to ESP32 (WROOM or S3)
2. Open Serial Monitor at **115200 baud**
3. The scanner runs automatically, repeating every 15 seconds

```
========================================
 Scan #1 (duration 8s)
========================================

Found 3 BLE devices:

  ----------------------------------------
  Device #2 *** BLE HID KEYBOARD ***
  ----------------------------------------
      MAC address:  aa:bb:cc:dd:ee:ff
      Address type: Random
      RSSI:         -61 dBm
      Name:         MyKeyboard
      Appearance:   Keyboard
      Services (1):
        [0] 1812 (HID - Human Interface Device)
      Connectable:  YES
```

Devices marked `*** BLE HID KEYBOARD ***` advertise an active HID service (UUID 0x1812). Use the MAC address from the output in the `connect` command.

## Settings

```cpp
#define SCAN_DURATION_SEC  8    // duration of one scan
#define SCAN_PAUSE_SEC    15    // pause between scans
```

---
