#!/usr/bin/env python3
"""
BLE-USB Bridge Console
======================
Automatically detects the ESP32-S3 CDC port and provides an interactive
menu-driven console for bridge configuration.

Supported firmware variants:
  - ble_usb_bridge_s3.ino          (keyboard only)
  - ble_usb_kb_mouse_bridge_s3.ino (keyboard + mouse)

Port detection: Espressif VID (0x303A) + description / product string.
If multiple candidates are found, an interactive selection is shown.

Usage:
  python ble_bridge_console.py           -- auto-detect port
  python ble_bridge_console.py COM3      -- specific port
  python ble_bridge_console.py --list    -- list all serial ports
"""

import sys
import os
import time
import threading
import argparse

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is not installed.")
    print("       Install with: pip install pyserial")
    sys.exit(1)

# ── Constants ─────────────────────────────────────────────────────────────────

APP_VERSION = "1.0"

# Espressif VID for ESP32-S3 TinyUSB CDC
ESPRESSIF_VID = 0x303A

# Known PIDs for ESP32-S3 with Arduino core + TinyUSB
ESP32S3_PIDS = {
    0x1001: "ESP32-S3 USB-OTG (TinyUSB)",
    0x4001: "ESP32-S3 TinyUSB CDC",
    0x0002: "ESP32-S3 CDC",
}

# Keywords matched against port description / manufacturer / product
BRIDGE_KEYWORDS = [
    "ble", "bridge", "esp32", "espressif", "tinyusb", "usb serial"
]

# Baud rate -- CDC ignores this value, but pyserial requires it
BAUD_RATE = 115200

# ── Port detection ────────────────────────────────────────────────────────────

def list_all_ports():
    """Return all available serial ports."""
    return list(serial.tools.list_ports.comports())


def score_port(port):
    """
    Return a likelihood score that this port is an ESP32-S3 bridge.
    Higher score = more likely.
    """
    score = 0
    desc = (port.description  or "").lower()
    mfr  = (port.manufacturer or "").lower()
    prod = (port.product      or "").lower()

    # VID/PID match -- strongest signal
    if port.vid == ESPRESSIF_VID:
        score += 100
        if port.pid in ESP32S3_PIDS:
            score += 50

    # Keywords in description / manufacturer / product
    for kw in BRIDGE_KEYWORDS:
        if kw in desc or kw in mfr or kw in prod:
            score += 10

    # Espressif manufacturer string
    if "espressif" in mfr:
        score += 30

    return score


def find_bridge_ports():
    """Return a sorted list of (port, score, info) tuples for candidate ports."""
    scored = []
    for p in list_all_ports():
        s = score_port(p)
        if s > 0:
            pid_name = ESP32S3_PIDS.get(p.pid, "") if p.pid else ""
            info = pid_name or p.description or p.device
            scored.append((p, s, info))
    scored.sort(key=lambda x: x[1], reverse=True)
    return scored


# ── ANSI colours ──────────────────────────────────────────────────────────────

USE_COLOR = (
    (sys.stdout.isatty() and os.name != "nt")
    or (os.name == "nt" and bool(os.environ.get("WT_SESSION")))  # Windows Terminal
)

def _c(code, text):
    return "\033[{}m{}\033[0m".format(code, text) if USE_COLOR else text

def cyan(t):    return _c("96", t)
def green(t):   return _c("92", t)
def yellow(t):  return _c("93", t)
def red(t):     return _c("91", t)
def bold(t):    return _c("1",  t)
def dim(t):     return _c("2",  t)


# ── Bridge console class ──────────────────────────────────────────────────────

class BridgeConsole:

    def __init__(self, port_name, variant="combo"):
        self.port_name   = port_name
        self.variant     = variant   # "keyboard" or "combo"
        self.ser         = None
        self._rx_thread  = None
        self._stop       = threading.Event()
        self._last_rx    = ""
        self._echo_filter = None   # last sent command — suppress echo line from firmware

    # ── Port open / close ──────────────────────────────────────────────────────

    def open(self):
        self.ser = serial.Serial(
            port     = self.port_name,
            baudrate = BAUD_RATE,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout  = 0.1,
            rtscts   = False,
            dsrdtr   = False,
        )
        # Assert DTR so firmware's USBCDC::operator bool() returns true.
        # Without this the firmware's conPrint() guard blocks CDC output.
        self.ser.dtr = True
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self):
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ── Send / receive ─────────────────────────────────────────────────────────

    def send(self, cmd):
        """Send a command to the bridge (appends newline)."""
        if not self.ser or not self.ser.is_open:
            print(red("  [!] Port is not open"))
            return
        try:
            self._echo_filter = cmd.strip().lower()
            self.ser.write((cmd + "\n").encode())
        except serial.SerialException as e:
            print(red("  [!] Send error: {}".format(e)))

    def _rx_loop(self):
        """Background thread: read lines from bridge and print them."""
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.ser.read(256)
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        decoded = line.decode("utf-8", errors="replace").rstrip()
                        if not decoded:
                            continue
                        # Suppress firmware echo of the last sent command
                        if self._echo_filter and decoded.strip().lower() == self._echo_filter:
                            self._echo_filter = None
                            continue
                        self._last_rx = decoded
                        self._print_rx(decoded)
            except serial.SerialException:
                if not self._stop.is_set():
                    print(red("\n  [!] Port disconnected."))
                    self._stop.set()
                break
            except Exception:
                break

    def _print_rx(self, line):
        """Print a received line with colour coding by prefix."""
        if line.startswith("[BLE]") or line.startswith("[DAEMON]"):
            print(cyan("  <- " + line))
        elif line.startswith("[SCAN]"):
            print(green("  <- " + line))
        elif line.startswith("[NVS]") or line.startswith("[CFG]"):
            print(yellow("  <- " + line))
        elif line.startswith("[ERR]"):
            print(red("  <- " + line))
        elif line.startswith("[KB]") or line.startswith("[HID]"):
            print(dim("  <- " + line))
        elif line.startswith("[MOUSE]") or line.startswith("[BTN]") or line.startswith("[MOVE]"):
            print(dim("  <- " + line))
        elif "---" in line or "===" in line:
            print(bold("  " + line))
        else:
            print("  " + line)

    def wait_rx(self, seconds=2.0):
        """Wait for bridge output to arrive."""
        time.sleep(seconds)

    # ── Input helpers ──────────────────────────────────────────────────────────

    def ask(self, prompt, default=""):
        """Prompt user for input with an optional default value."""
        if default:
            val = input("  {} [{}]: ".format(prompt, default)).strip()
            return val if val else default
        return input("  {}: ".format(prompt)).strip()

    def ask_mac(self, label="MAC address"):
        """Ask for a BLE MAC address in xx:xx:xx:xx:xx:xx format."""
        while True:
            mac = self.ask(label).strip().lower()
            if len(mac) == 17 and mac.count(":") == 5:
                return mac
            # Accept MAC without separators (12 hex chars)
            clean = mac.replace(":", "").replace("-", "").replace(" ", "")
            if len(clean) == 12 and all(c in "0123456789abcdef" for c in clean):
                formatted = ":".join(clean[i:i+2] for i in range(0, 12, 2))
                confirm = self.ask("  Did you mean {}? (y/n)".format(formatted), "y")
                if confirm.lower() == "y":
                    return formatted
            print(red("  [!] Invalid format. Example: aa:bb:cc:dd:ee:ff"))

    # ── Header ─────────────────────────────────────────────────────────────────

    def print_header(self):
        os.system("cls" if os.name == "nt" else "clear")
        print(bold(cyan("============================================")))
        print(bold(cyan("  BLE-USB Bridge Console  v{}".format(APP_VERSION))))
        print(bold(cyan("============================================")))
        variant_label = "Keyboard + Mouse" if self.variant == "combo" else "Keyboard only"
        print("  Port:    {}".format(green(self.port_name)))
        print("  Variant: {}".format(green(variant_label)))
        print()

    # ── Main menu ──────────────────────────────────────────────────────────────

    def main_menu(self):
        while True:
            self.print_header()
            print(bold("  MAIN MENU"))
            print()
            print("  {}  Keyboard -- connect / disconnect".format(bold("1")))
            if self.variant == "combo":
                print("  {}  Mouse -- connect / settings".format(bold("2")))
            print("  {}  Show status".format(bold("3")))
            print("  {}  Direct command line".format(bold("4")))
            print("  {}  Help (bridge commands)".format(bold("5")))
            print()
            print("  {}  Quit".format(bold("q")))
            print()

            choice = input("  Choice: ").strip().lower()

            if choice == "1":
                self.menu_keyboard()
            elif choice == "2" and self.variant == "combo":
                self.menu_mouse()
            elif choice == "3":
                self.do_status()
            elif choice == "4":
                self.direct_console()
            elif choice == "5":
                self.do_help()
            elif choice in ("q", "quit", "exit"):
                break

    # ── Keyboard menu ──────────────────────────────────────────────────────────

    def menu_keyboard(self):
        while True:
            self.print_header()
            print(bold("  KEYBOARD"))
            print()
            print("  {}  Scan for BLE devices".format(bold("1")))
            print("  {}  Connect keyboard  (connect kb)".format(bold("2")))
            print("  {}  Forget keyboard   (forget kb)".format(bold("3")))
            print("  {}  Status".format(bold("4")))
            print()
            print("  {}  Back".format(bold("b")))
            print()

            choice = input("  Choice: ").strip().lower()

            if choice == "1":
                self._do_scan()
            elif choice == "2":
                self._do_connect_kb()
            elif choice == "3":
                self._do_forget("kb")
            elif choice == "4":
                self.do_status()
            elif choice in ("b", "back"):
                break

    # ── Mouse menu ─────────────────────────────────────────────────────────────

    def menu_mouse(self):
        while True:
            self.print_header()
            print(bold("  MOUSE"))
            print()
            print("  {}  Scan for BLE devices".format(bold("1")))
            print("  {}  Connect mouse      (connect mouse)".format(bold("2")))
            print("  {}  Forget mouse       (forget mouse)".format(bold("3")))
            print("  {}  Set scale          (DPI divisor)".format(bold("4")))
            print("  {}  Toggle Y inversion (flipy)".format(bold("5")))
            print("  {}  Toggle scroll inversion (flipw)".format(bold("6")))
            print("  {}  Set Report ID filter".format(bold("7")))
            print("  {}  Status".format(bold("8")))
            print()
            print("  {}  Back".format(bold("b")))
            print()

            choice = input("  Choice: ").strip().lower()

            if choice == "1":
                self._do_scan()
            elif choice == "2":
                self._do_connect_mouse()
            elif choice == "3":
                self._do_forget("mouse")
            elif choice == "4":
                self._do_scale()
            elif choice == "5":
                self._do_flipy()
            elif choice == "6":
                self._do_flipw()
            elif choice == "7":
                self._do_reportid()
            elif choice == "8":
                self.do_status()
            elif choice in ("b", "back"):
                break

    # ── Actions ────────────────────────────────────────────────────────────────

    def _do_scan(self):
        print()
        print(yellow("  -> Starting scan (10 s)..."))
        self.send("scan")
        print(dim("  Put your BLE device into pairing mode now."))
        print(dim("  Results appear as devices are discovered. Please wait..."))
        self.wait_rx(11.0)
        input(dim("\n  [Enter] to continue..."))

    def _do_connect_kb(self):
        print()
        mac = self.ask_mac("Keyboard MAC address")
        cmd = "connect {}".format(mac) if self.variant == "keyboard" else "connect kb {}".format(mac)
        print(yellow("  -> {}".format(cmd)))
        self.send(cmd)
        self.wait_rx(15.0)
        input(dim("\n  [Enter] to continue..."))

    def _do_connect_mouse(self):
        print()
        mac = self.ask_mac("Mouse MAC address")
        print(yellow("  -> connect mouse {}".format(mac)))
        self.send("connect mouse {}".format(mac))
        self.wait_rx(15.0)
        input(dim("\n  [Enter] to continue..."))

    def _do_forget(self, what="kb"):
        print()
        if what == "kb":
            cmd = "forget" if self.variant == "keyboard" else "forget kb"
        elif what == "mouse":
            cmd = "forget mouse"
        else:
            cmd = "forget all"
        confirm = self.ask("Really run '{}'? (y/n)".format(cmd), "n")
        if confirm.lower() == "y":
            print(yellow("  -> {}".format(cmd)))
            self.send(cmd)
            self.wait_rx(2.0)
        else:
            print(dim("  Cancelled."))
        input(dim("\n  [Enter] to continue..."))

    def _do_scale(self):
        print()
        print(dim("  Recommended values by mouse DPI:"))
        print(dim("     400 DPI  ->  scale 1"))
        print(dim("     800 DPI  ->  scale 2"))
        print(dim("    1600 DPI  ->  scale 4  (default)"))
        print(dim("    3200 DPI  ->  scale 8"))
        print()
        val = self.ask("Scale value (1-64)", "4")
        try:
            n = int(val)
            if 1 <= n <= 64:
                print(yellow("  -> scale {}".format(n)))
                self.send("scale {}".format(n))
                self.wait_rx(1.5)
            else:
                print(red("  [!] Value must be between 1 and 64"))
        except ValueError:
            print(red("  [!] Invalid value"))
        input(dim("\n  [Enter] to continue..."))

    def _do_flipy(self):
        print()
        print(yellow("  -> flipy"))
        self.send("flipy")
        self.wait_rx(1.5)
        input(dim("\n  [Enter] to continue..."))

    def _do_flipw(self):
        print()
        print(yellow("  -> flipw"))
        self.send("flipw")
        self.wait_rx(1.5)
        input(dim("\n  [Enter] to continue..."))

    def _do_reportid(self):
        print()
        print(dim("  Examples:"))
        print(dim("    0         -- auto (default, tries all Input reports)"))
        print(dim("    17 (0x11) -- Logitech MX Master 2/3"))
        print(dim("    1 or 3    -- most standard Bluetooth mice"))
        print()
        val = self.ask("Report ID (0-255)", "0")
        try:
            n = int(val)
            if 0 <= n <= 255:
                print(yellow("  -> reportid {}".format(n)))
                self.send("reportid {}".format(n))
                self.wait_rx(1.5)
                print(dim("  Takes effect after reconnecting the mouse."))
            else:
                print(red("  [!] Value must be between 0 and 255"))
        except ValueError:
            print(red("  [!] Invalid value"))
        input(dim("\n  [Enter] to continue..."))

    def do_status(self):
        print()
        print(yellow("  -> status"))
        self.send("status")
        self.wait_rx(2.0)
        input(dim("\n  [Enter] to continue..."))

    def do_help(self):
        print()
        print(yellow("  -> help"))
        self.send("help")
        self.wait_rx(2.0)
        input(dim("\n  [Enter] to continue..."))

    def direct_console(self):
        """Direct command line -- send any raw command to the bridge."""
        print()
        print(bold("  DIRECT CONSOLE"))
        print(dim("  Type commands and press Enter."))
        print(dim("  Type 'exit', 'q' or 'back' to return to the menu."))
        print()
        while True:
            try:
                line = input(cyan("  bridge> ")).strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            if line.lower() in ("exit", "q", "quit", "back"):
                break
            self.send(line)
            time.sleep(0.5)


# ── Interactive port selection ────────────────────────────────────────────────

def select_port_interactive():
    """
    Interactively select a port when multiple candidates exist.
    Returns the port device name or None.
    """
    candidates = find_bridge_ports()
    all_ports  = list_all_ports()

    if not candidates and not all_ports:
        print(red("  [!] No serial ports found."))
        return None

    # Only one strong candidate -- use it directly
    if len(candidates) == 1:
        p, score, info = candidates[0]
        print(green("  Port found: {}  ({})".format(p.device, info)))
        return p.device

    print(bold("\n  Available ports:"))
    print()

    all_listed = []

    if candidates:
        print(bold("  Likely ESP32-S3 bridge ports:"))
        for i, (p, score, info) in enumerate(candidates, 1):
            tag = green("[{}]".format(i))
            print("  {}  {:<12}  {}".format(tag, p.device, info))
            all_listed.append(p.device)
        print()

    others = [p for p in all_ports if p.device not in all_listed]
    if others:
        print(bold("  Other ports:"))
        for i, p in enumerate(others, len(all_listed) + 1):
            tag = dim("[{}]".format(i))
            desc = p.description or p.device
            print("  {}  {:<12}  {}".format(tag, p.device, desc))
            all_listed.append(p.device)
        print()

    if not all_listed:
        print(red("  No ports found."))
        return None

    choice = input("  Select port number (or type name, Enter = 1): ").strip()

    if not choice:
        return all_listed[0]

    try:
        idx = int(choice) - 1
        if 0 <= idx < len(all_listed):
            return all_listed[idx]
    except ValueError:
        for dev in all_listed:
            if choice.upper() == dev.upper():
                return dev

    print(red("  [!] Invalid selection."))
    return None


def detect_variant(port_name):
    """
    Try to detect the bridge variant (keyboard / combo) from the USB product string.
    Defaults to 'combo' -- it supports all commands.
    """
    for p in list_all_ports():
        if p.device == port_name:
            prod = (p.product or "").lower()
            desc = (p.description or "").lower()
            if "mouse" in prod or "mouse" in desc or "combo" in prod:
                return "combo"
            break
    return "combo"


# ── CLI entry point ───────────────────────────────────────────────────────────

def cmd_list():
    """Print all serial ports with bridge candidates highlighted."""
    all_ports      = list_all_ports()
    candidates     = find_bridge_ports()
    candidate_devs = {p.device for p, _, _ in candidates}

    print(bold("\n  Serial ports:"))
    print()
    if not all_ports:
        print(dim("  (none)"))
        return

    for p in all_ports:
        star = green(" *") if p.device in candidate_devs else "  "
        vid  = "VID={:04X}".format(p.vid) if p.vid else "      "
        pid  = "PID={:04X}".format(p.pid) if p.pid else "      "
        desc = p.description or "-"
        print("{} {:<12}  {} {}  {}".format(star, p.device, vid, pid, desc))

    if candidates:
        print()
        print(dim("  * = likely ESP32-S3 bridge port"))
    print()


def main():
    parser = argparse.ArgumentParser(
        description="BLE-USB Bridge Console",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "port", nargs="?",
        help="Serial port (e.g. COM3 or /dev/ttyACM0)"
    )
    parser.add_argument(
        "--list", "-l", action="store_true",
        help="List available serial ports and exit"
    )
    parser.add_argument(
        "--variant", choices=["keyboard", "combo"], default=None,
        help="Firmware variant (keyboard / combo). Default: auto-detect"
    )
    args = parser.parse_args()

    print(bold(cyan("\n  BLE-USB Bridge Console v{}".format(APP_VERSION))))
    print()

    if args.list:
        cmd_list()
        return

    # ── Port selection ────────────────────────────────────────────────────────
    port_name = args.port
    if not port_name:
        port_name = select_port_interactive()

    if not port_name:
        print(red("  [!] No port selected. Exiting."))
        sys.exit(1)

    # ── Variant detection ─────────────────────────────────────────────────────
    variant = args.variant or detect_variant(port_name)
    print(dim("  Variant: {}".format(variant)))

    # ── Open port ─────────────────────────────────────────────────────────────
    console = BridgeConsole(port_name, variant)
    try:
        console.open()
        print(green("  Port {} opened ({} baud, 8N1)".format(port_name, BAUD_RATE)))
        time.sleep(0.5)
    except serial.SerialException as e:
        print(red("  [!] Cannot open port {}: {}".format(port_name, e)))
        sys.exit(1)

    # ── Run menu ──────────────────────────────────────────────────────────────
    try:
        console.main_menu()
    except KeyboardInterrupt:
        print()
        print(yellow("  Interrupted."))
    finally:
        console.close()
        print(green("\n  Port {} released.".format(port_name)))
        print()


if __name__ == "__main__":
    main()
