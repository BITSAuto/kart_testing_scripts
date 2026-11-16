#!/usr/bin/env python3
"""USB serial devices on this machine, and udev rules for the kart (standard library only).

    python3 tools/serial_devices.py list                     table of ttyUSB*/ttyACM* devices
    python3 tools/serial_devices.py list --json
    python3 tools/serial_devices.py rules --dbw /dev/ttyUSB0 [--vn /dev/ttyUSB1]
                                                              print a rules file for them
    python3 tools/serial_devices.py generic-rules            rules for known chips only

The rules give stable names, independent of plug-in order:
    /dev/kart_dbw         drive-by-wire ESP32
    /dev/kart_vectornav   VectorNav
plus group dialout / mode 0660, and ID_MM_DEVICE_IGNORE so ModemManager doesn't send AT
commands to them (it probes every new serial port, which can garble or reset an ESP32).

Matching: vendor + product + serial number when the serial number is unique. Cheap chips
(CH340) have no serial number, and an ESP32 and a VectorNav cable can both be FTDI 0403:6001,
so if that doesn't tell them apart the rule matches the physical USB port instead (then that
device must always go in the same USB socket).
"""
import argparse
import glob
import json
import os
import sys

KNOWN = {
    ("10c4", "ea60"): "Silicon Labs CP210x (common on ESP32 boards)",
    ("1a86", "7523"): "WCH CH340 (common on ESP32 boards)",
    ("1a86", "55d4"): "WCH CH9102 (ESP32 boards)",
    ("1a86", "7522"): "WCH CH340K",
    ("0403", "6001"): "FTDI FT232R (VectorNav cables, some ESP32 boards)",
    ("0403", "6015"): "FTDI FT231X",
    ("0403", "6014"): "FTDI FT232H",
    ("303a", "1001"): "Espressif native USB (ESP32-S2/S3/C3/C6)",
    ("303a", "4001"): "Espressif native USB CDC",
}
RULES_PATH = "/etc/udev/rules.d/70-kart-serial.rules"


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def devices():
    out = []
    for node in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        name = os.path.basename(node)
        dev = os.path.realpath(f"/sys/class/tty/{name}/device")
        info = {"node": node, "vid": None, "pid": None, "serial": None, "manufacturer": None,
                "product": None, "usb_port": None, "interface": None, "by_id": []}
        p = dev
        while p and p != "/":
            if info["interface"] is None and os.path.exists(os.path.join(p, "bInterfaceNumber")):
                info["interface"] = _read(os.path.join(p, "bInterfaceNumber"))
            if os.path.exists(os.path.join(p, "idVendor")):
                info["vid"] = _read(os.path.join(p, "idVendor"))
                info["pid"] = _read(os.path.join(p, "idProduct"))
                info["serial"] = _read(os.path.join(p, "serial"))
                info["manufacturer"] = _read(os.path.join(p, "manufacturer"))
                info["product"] = _read(os.path.join(p, "product"))
                info["usb_port"] = os.path.basename(p)
                break
            p = os.path.dirname(p)
        for link in glob.glob("/dev/serial/by-id/*"):
            if os.path.realpath(link) == os.path.realpath(node):
                info["by_id"].append(link)
        info["chip"] = KNOWN.get((info["vid"], info["pid"]), "unknown chip")
        for alias in ("/dev/kart_dbw", "/dev/kart_vectornav"):
            if os.path.exists(alias) and os.path.realpath(alias) == os.path.realpath(node):
                info.setdefault("aliases", []).append(alias)
        out.append(info)
    return out


def find(node):
    real = os.path.realpath(node)
    for d in devices():
        if os.path.realpath(d["node"]) == real:
            return d
    sys.exit(f"{node}: not a USB serial device on this machine (plugged in?)")


def match_for(d, others):
    """udev match keys that identify d among the devices in others."""
    keys = [f'ATTRS{{idVendor}}=="{d["vid"]}"', f'ATTRS{{idProduct}}=="{d["pid"]}"']
    clash = [o for o in others if o is not d and (o["vid"], o["pid"]) == (d["vid"], d["pid"])
             and o.get("serial") == d.get("serial")]
    how = "vendor+product"
    if d.get("serial") and not clash:
        keys.append(f'ATTRS{{serial}}=="{d["serial"]}"')
        how = "vendor+product+serial number"
    elif clash or not d.get("serial"):
        keys.append(f'KERNELS=="{d["usb_port"]}"')
        how = f"USB port {d['usb_port']} (keep it plugged into the same socket)"
    if d.get("interface") not in (None, "00"):
        keys.append(f'ENV{{ID_USB_INTERFACE_NUM}}=="{d["interface"]}"')
    return ", ".join(keys), how


def generic_rules():
    lines = ["# Kart test bench: keep ModemManager off common USB-serial chips, dialout access."]
    for (vid, pid), name in KNOWN.items():
        lines.append(f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{vid}", ATTRS{{idProduct}}=="{pid}", '
                     f'MODE="0660", GROUP="dialout", ENV{{ID_MM_DEVICE_IGNORE}}="1"   # {name}')
    return "\n".join(lines) + "\n"


def role_rules(dbw=None, vn=None):
    devs = devices()
    chosen = [(n, find(p)) for n, p in (("kart_dbw", dbw), ("kart_vectornav", vn)) if p]
    if len(chosen) == 2 and chosen[0][1]["node"] == chosen[1][1]["node"]:
        sys.exit("the drive-by-wire and VectorNav ports are the same device")
    lines = [generic_rules().rstrip(), "", "# Stable names (written by setup.sh --udev)"]
    notes = []
    for name, d in chosen:
        keys, how = match_for(d, devs)
        lines.append(f'SUBSYSTEM=="tty", {keys}, SYMLINK+="{name}", MODE="0660", GROUP="dialout", '
                     f'ENV{{ID_MM_DEVICE_IGNORE}}="1"')
        notes.append(f"/dev/{name} <- {d['node']} ({d['chip']}), matched by {how}")
    return "\n".join(lines) + "\n", notes


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    ls = sub.add_parser("list")
    ls.add_argument("--json", action="store_true")
    r = sub.add_parser("rules")
    r.add_argument("--dbw")
    r.add_argument("--vn")
    r.add_argument("--notes", help="write the human-readable notes to this file")
    sub.add_parser("generic-rules")
    a = ap.parse_args()
    if a.cmd == "list":
        devs = devices()
        if a.json:
            print(json.dumps(devs, indent=2))
            return
        if not devs:
            print("no USB serial devices (/dev/ttyUSB*, /dev/ttyACM*) found")
            return
        for i, d in enumerate(devs, 1):
            al = f"  [{', '.join(d['aliases'])}]" if d.get("aliases") else ""
            print(f"{i}) {d['node']}  {d['vid']}:{d['pid']}  {d['chip']}{al}")
            print(f"     {d['manufacturer'] or '?'} / {d['product'] or '?'}  serial "
                  f"{d['serial'] or '(none)'}  usb port {d['usb_port']}")
    elif a.cmd == "generic-rules":
        sys.stdout.write(generic_rules())
    else:
        if not (a.dbw or a.vn):
            sys.exit("give --dbw and/or --vn")
        text, notes = role_rules(a.dbw, a.vn)
        sys.stdout.write(text)
        if a.notes:
            with open(a.notes, "w") as f:
                f.write("\n".join(notes) + "\n")


if __name__ == "__main__":
    main()
