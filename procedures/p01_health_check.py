#!/usr/bin/env python3
"""p01 health check: is everything ready to test?

Checks, each reported as ok / warn / fail:
  python packages, data folder, serial port (exists, permissions, free), data arriving,
  encoder lines parsed, encoder still at rest, calibration files and their age, encoder zero
  still plausible (ESP32 restarts reset the count), VectorNav data / fix / at rest,
  and with --motion-check: a short pulse left and right moves the encoder the right way.

Results: data/01_health_check/<run>/checks.csv and summary.json.
"""
import glob
import grp
import json
import os
import shutil
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.config import CALIBRATIONS  # noqa: E402
from kart.harness import run, run_trial  # noqa: E402


def add_args(ap):
    ap.add_argument("--motion-check", action="store_true",
                    help="pulse the steering left and right (needs SPACE)")
    ap.add_argument("--listen", type=float, default=2.0)


def body(ctx):
    ch, link, cfg, cal, rec = ctx.ch, ctx.link, ctx.cfg, ctx.cal, ctx.rec
    rec.table("checks", ["name", "status", "detail"])
    results = []

    def check(name, status, detail=""):
        results.append({"name": name, "status": status, "detail": detail})
        rec.row("checks", [name, status, detail])
        ch.emit("check", name=name, status=status, detail=detail)
        ch.log(f"{name}: {detail}", {"ok": "ok", "warn": "warn", "fail": "error"}[status])

    # ---------------------------------------------------------- environment
    try:
        import numpy
        import serial
        check("python packages", "ok", f"pyserial {serial.__version__}, numpy {numpy.__version__}")
    except ImportError as e:
        check("python packages", "fail", str(e))
    root = Path(cfg["_data_root"])
    free_gb = shutil.disk_usage(root).free / 1e9
    writable = os.access(root, os.W_OK)
    check("data folder", "ok" if writable and free_gb > 1 else ("warn" if writable else "fail"),
          f"{root} ({free_gb:.1f} GB free{'' if writable else ', NOT writable'})")

    # ------------------------------------------------------------ serial port
    port = cfg["serial"]["port"]
    if not os.path.exists(port):
        found = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        check("serial port", "fail", f"{port} does not exist. Present: {found or 'none'}")
        return finish(results)
    real = os.path.realpath(port)
    if not os.access(real, os.R_OK | os.W_OK):
        groups = [grp.getgrgid(g).gr_name for g in os.getgroups()]
        check("serial port", "fail", f"no read/write permission on {real}. Fix: sudo usermod -aG "
              f"dialout $USER, then log out and in (your groups: {groups})")
        return finish(results)
    try:
        link.open(send_neutral=False)
        check("serial port", "ok", f"{port} -> {real} opened at {cfg['serial']['baud']} baud")
    except Exception as e:
        busy = "busy" in str(e).lower() or "lock" in str(e).lower()
        check("serial port", "fail", f"cannot open {port}: {e}" +
              (" (another program has it open: ver1.py? the monitor?)" if busy else ""))
        return finish(results)

    # -------------------------------------------------------------- data flow
    n0, e0 = link.rx_count, link.enc_count
    time.sleep(ctx.args.listen)
    rate = (link.rx_count - n0) / ctx.args.listen
    enc_rate = (link.enc_count - e0) / ctx.args.listen
    if rate == 0:
        link.send(note="health neutral frame")
        time.sleep(1.5)
        after = link.rx_count - n0
        if after:
            check("data from ESP32", "warn", "silent until a frame is sent: set [serial] "
                  "poll_interval_s = 0.1 (or check the firmware)")
        else:
            check("data from ESP32", "fail", "nothing received, even after a frame. Wrong port or "
                  "baud? ESP32 powered? Run p00_serial_probe.")
            return finish(results)
    else:
        check("data from ESP32", "ok", f"{rate:.1f} lines/s")
        link.send(note="health neutral frame")
    last = link.last_lines[-1][2] if link.last_lines else ""
    if link.enc_count == 0:
        check("encoder parsing", "fail", f"regex matched no line. Last line: '{last[:60]}'. "
              "Run p00_serial_probe --pulse 40 to find the field.")
    else:
        check("encoder parsing", "ok" if enc_rate >= 10 else "warn",
              f"{enc_rate:.1f} samples/s (raw {link.enc[1]})" +
              ("" if enc_rate >= 10 else " - slow: latency measurements will be coarse"))
        s = link.samples_between(time.monotonic() - 1.0)
        if s:
            spread = max(r for _, r in s) - min(r for _, r in s)
            check("encoder at rest", "ok" if spread <= ctx.cfg["encoder"]["noise_ticks"] else "warn",
                  f"spread {spread} ticks over 1 s with steering stopped")

    # ------------------------------------------------------------ calibration
    optional = {"slew_rolling", "step_steer"}
    have = [n for n in CALIBRATIONS if cal.has(n)]
    missing = [n for n in CALIBRATIONS if not cal.has(n) and n not in optional]
    check("calibration files", "ok" if not missing else "warn",
          f"have {have or 'none'}; missing {missing or 'none'}")
    if cal.has("zero") and link.enc[1] is not None:
        z = cal.get("zero")
        age_h = (time.time() - time.mktime(time.strptime(z["saved_at"], "%Y-%m-%d %H:%M:%S"))) / 3600
        pos = cal.pos(link.enc[1])
        lim = cal.soft_limits()
        span = (lim[1] - lim[0]) if lim else None
        plausible = span is None or abs(pos) < span
        status = "ok" if plausible and age_h < 12 else "warn"
        check("encoder zero", status,
              f"set {age_h:.1f} h ago; wheel now at pos {pos}" +
              ("" if plausible else " - outside the lock range: ESP32 restarted? Re-zero (p03 "
               "--zero-only)") + (" - old, re-zero if the ESP32 was power cycled" if age_h >= 12
                                 else ""))
    elif not cal.has("zero"):
        check("encoder zero", "warn", "not set: run p03")

    # -------------------------------------------------------------- VectorNav
    src = cfg["vectornav"]["source"]
    if src == "none":
        check("VectorNav", "warn", "source = none: p09/p10 (speed, circles) need it")
    else:
        t0 = time.monotonic()
        while not ctx.vn.available() and time.monotonic() - t0 < 3:
            time.sleep(0.1)
        if not ctx.vn.available():
            check("VectorNav", "fail", f"source {src}: no data")
        else:
            c0 = ctx.vn.count
            time.sleep(1.0)
            st = ctx.vn.state
            v = (st.get("speed_mps") or 0) * 3.6
            check("VectorNav", "ok" if st.get("fix") else "warn",
                  f"{ctx.vn.count - c0:.0f} msgs/s, fix={st.get('fix')}, speed {v:.2f} km/h"
                  + ("" if st.get("fix") else " - no GNSS fix yet"))

    # ------------------------------------------------------------ motion check
    if ctx.args.motion_check:
        if link.enc_count == 0:
            check("steering moves", "fail", "skipped: no encoder data")
        else:
            moved = {}
            for d in ("left", "right"):
                def pulse():
                    r0 = link.enc[1]
                    ctx.steer.command(d, note=f"health pulse {d}")
                    ctx.sleep(0.3)
                    ctx.steer.stop(note="health pulse stop", force=False)
                    return link.enc[1] - r0
                moved[d] = run_trial(ctx, f"health pulse {d}", pulse)
            ok_move = all(abs(v) > 2 for v in moved.values()) and \
                moved["left"] * moved["right"] < 0
            sign_ok = (moved["left"] * cal.sign > 0) if cal.has("steer_map") else None
            detail = f"raw change left {moved['left']}, right {moved['right']}"
            if not ok_move:
                check("steering moves", "fail", detail + " - expected opposite movements")
            elif sign_ok is False:
                check("steering moves", "fail", detail + " - LEFT moved the wrong way for "
                      "steer_map.json: rerun p02")
            else:
                check("steering moves", "ok", detail)
    return finish(results)


def finish(results):
    worst = "ok"
    for r in results:
        if r["status"] == "fail":
            worst = "fail"
        elif r["status"] == "warn" and worst == "ok":
            worst = "warn"
    return {"overall": worst, "checks": results}


if __name__ == "__main__":
    run("01_health_check", "Is everything ready to test?", body, add_args,
        motion=lambda a: a.motion_check, open_link=False, send_neutral=False)
