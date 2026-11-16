#!/usr/bin/env python3
"""One text snapshot of everything needed to debug the bench in the field.

    .venv/bin/python tools/diagnose.py              # print
    .venv/bin/python tools/diagnose.py --listen 3   # also listen to the port for 3 s (sends nothing)
    .venv/bin/python tools/diagnose.py --out diag.txt

Sections: system, Python/packages, config, serial devices + udev/brltty/ModemManager, port
check (+ optional passive listen with regex match rate), calibration files, the last runs
(status, error, last events, last frames sent, last lines received), self-test result.
Read-only: never sends anything to the kart.
"""
import argparse
import csv
import glob
import io
import json
import os
import platform
import subprocess
import sys
import time
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10).stdout.strip()
    except Exception as e:
        return f"({e})"


def tail_csv(p, n):
    try:
        rows = list(csv.reader(open(p, newline="")))
        return rows[:1] + rows[-n:] if len(rows) > n + 1 else rows
    except OSError:
        return []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config")
    ap.add_argument("--listen", type=float, default=0.0)
    ap.add_argument("--runs", type=int, default=4, help="how many recent runs to show")
    ap.add_argument("--out")
    a = ap.parse_args()
    buf = io.StringIO()
    with redirect_stdout(buf):
        print(f"# kart bench diagnose  {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print("\n## system")
        print(f"host {platform.node()}  {platform.platform()}  python {platform.python_version()} "
              f"({sys.executable})")
        print(f"user groups: {sh('id -nG')}")
        print(f"ROS_DISTRO={os.environ.get('ROS_DISTRO')} ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}")
        print(f"TERM={os.environ.get('TERM')} size={sh('stty size 2>/dev/null')} SSH={bool(os.environ.get('SSH_CONNECTION'))}")
        print("\n## packages")
        for m in ("serial", "numpy", "textual", "rich", "tomli", "rclpy"):
            try:
                mod = __import__(m)
                print(f"{m} {getattr(mod, '__version__', 'ok')}")
            except Exception as e:
                print(f"{m} MISSING ({type(e).__name__})")
        print("\n## config")
        try:
            from kart.config import Calibration, load_config
            cfg = load_config(a.config)
            print(f"file {cfg['_path']}  data {cfg['_data_root']}")
            for sec in ("serial", "encoder", "steering", "throttle", "safety", "vectornav", "vehicle"):
                print(f"[{sec}] {json.dumps(cfg.get(sec), default=str)}")
        except Exception as e:
            print(f"CONFIG ERROR: {e}")
            cfg = None
        print("\n## serial devices / host")
        print(sh(f"{sys.executable} {ROOT / 'tools' / 'serial_devices.py'} list"))
        print("by-id:", sh("ls -l /dev/serial/by-id/ 2>/dev/null | tail -n +2") or "(none)")
        print("aliases:", sh("ls -l /dev/kart_* 2>/dev/null") or "(none)")
        print("udev rules:", sh("cat /etc/udev/rules.d/70-kart-serial.rules 2>/dev/null | grep -v '^#' | grep SYMLINK") or "(no named rules)")
        print("brltty:", sh("dpkg -s brltty 2>/dev/null | grep Status") or "not installed",
              "| mask:", sh("readlink /etc/udev/rules.d/85-brltty.rules 2>/dev/null") or "none")
        print("ModemManager:", sh("systemctl is-active ModemManager 2>/dev/null"))
        print("kernel (last USB serial events):")
        print(sh("journalctl -k -n 400 --no-pager 2>/dev/null | grep -iE 'ttyUSB|ttyACM|cp210|ch34|ftdi|brltty|disconnect' | tail -12")
              or sh("dmesg 2>/dev/null | grep -iE 'ttyUSB|ttyACM|cp210|ch34|ftdi|brltty' | tail -12") or "(no access)")
        print("processes using serial ports:", sh("fuser -v /dev/ttyUSB* /dev/ttyACM* 2>&1 | tail -n +2") or "none")
        if cfg:
            port = cfg["serial"]["port"]
            real = os.path.realpath(port)
            print(f"\n## port {port} -> {real}")
            print("exists" if os.path.exists(port) else "MISSING",
                  "| rw ok" if os.access(real, os.R_OK | os.W_OK) else "| NO rw permission")
            if a.listen > 0 and os.path.exists(port):
                import re
                import serial
                rx = re.compile(cfg["encoder"]["regex"])
                try:
                    s = serial.Serial(port, cfg["serial"]["baud"], timeout=0.1, exclusive=True)
                    t0, lines = time.time(), []
                    while time.time() - t0 < a.listen:
                        ln = s.readline()
                        if ln:
                            lines.append(ln.decode(errors="replace").strip())
                    s.close()
                    m = sum(bool(rx.search(x)) for x in lines)
                    print(f"listened {a.listen}s: {len(lines)} lines ({len(lines) / a.listen:.1f}/s), "
                          f"encoder regex matched {m}")
                    for x in lines[-8:]:
                        print("   ", repr(x))
                    if not lines:
                        print("   silent: wrong port/baud, ESP32 off, or it only answers frames "
                              "(poll_interval_s)")
                except Exception as e:
                    print(f"listen failed: {e}")
            print("\n## calibration")
            cal = Calibration(cfg)
            for name, d in cal.data.items():
                keep = {k: v for k, v in d.items() if k in (
                    "saved_at", "left_value", "right_value", "left_ticks_sign", "mode", "raw_ticks",
                    "lock_left_pos", "lock_right_pos", "lock_left_deg", "lock_right_deg",
                    "ticks_per_deg", "linear_r2", "left_over_right", "backlash_deg",
                    "smallest_gap_without_drops_s", "wheelbase_m")}
                for side in ("left", "right"):
                    if isinstance(d.get(side), dict):
                        keep[side] = {k: v for k, v in d[side].items() if not isinstance(v, (dict, list))}
                print(f"{name}: {json.dumps(keep, default=str)}")
            print(f"soft limits (pos ticks): {cal.soft_limits()}")
            print("\n## recent runs")
            runs = sorted((p for p in Path(cfg["_data_root"]).glob("*/*/meta.json")),
                          key=lambda p: p.stat().st_mtime)[-a.runs:]
            for mp in runs:
                d = mp.parent
                meta = json.loads(mp.read_text())
                summ = {}
                if (d / "summary.json").exists():
                    summ = json.loads((d / "summary.json").read_text())
                print(f"\n### {d.relative_to(cfg['_data_root'])}  status={meta.get('status')} "
                      f"duration={meta.get('duration_s')}s args={' '.join(meta.get('argv', [])[1:])}")
                if summ.get("error"):
                    print(f"ERROR: {summ['error']}")
                for r in tail_csv(d / "events.csv", 8)[1:]:
                    print("  event", r[0], r[2], r[3][:120])
                for r in tail_csv(d / "serial_tx.csv", 5)[1:]:
                    print("  tx", r[0], r[2], r[-1])
                for r in tail_csv(d / "serial_rx.csv", 4)[1:]:
                    print("  rx", r[0], r[3], r[4][:80])
            st = sorted(glob.glob(str(Path(cfg["_data_root"]) / "self_test" / "*.json")))
            if st:
                rep = json.loads(Path(st[-1]).read_text())
                print(f"\n## last self-test {Path(st[-1]).name}: {rep['counts']}")
                for r in rep["results"]:
                    if r["status"] in ("FAIL", "WARN"):
                        print(f"  {r['status']} {r['check']}: {r['detail']}")
    text = buf.getvalue()
    if a.out:
        Path(a.out).write_text(text)
        print(f"written to {a.out}")
    else:
        print(text)


if __name__ == "__main__":
    main()
