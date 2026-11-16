#!/usr/bin/env python3
"""Passive live view for the dashboard when no test is running.

Reads the drive-by-wire port and VectorNav and streams telemetry. Sends nothing (unless
[serial] poll_interval_s is set, in which case it re-sends a neutral frame to keep data
coming). Records nothing. The dashboard stops it before starting a procedure.
"""
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.config import Calibration, load_config  # noqa: E402
from kart.link import DbwLink  # noqa: E402
from kart.vectornav import make_vectornav  # noqa: E402


def out(kind, **d):
    sys.stdout.write("@" + json.dumps({"k": kind, **d}, default=str) + "\n")
    sys.stdout.flush()


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--config")
    ap.add_argument("--tui", action="store_true")
    args = ap.parse_args()
    cfg = load_config(args.config)
    cal = Calibration(cfg)
    link = DbwLink(cfg, None, print)
    try:
        link.open(send_neutral=cfg["serial"].get("poll_interval_s", 0) > 0)
    except Exception as e:
        print(f"[ERROR] monitor: cannot open {cfg['serial']['port']}: {e}", flush=True)
        out("monitor", ok=False, error=str(e))
        return 1
    vn = make_vectornav(cfg)
    try:
        vn.start()
    except Exception as e:
        print(f"[WARN] monitor: VectorNav not started: {e}", flush=True)
    out("monitor", ok=True, port=cfg["serial"]["port"])
    print(f"monitor: listening on {cfg['serial']['port']} (sending nothing)", flush=True)
    t0 = time.monotonic()
    last = (t0, 0, 0, 0)
    hz = (0, 0, 0)
    try:
        while True:
            time.sleep(0.05)
            now = time.monotonic()
            if now - last[0] >= 1:
                dt = now - last[0]
                hz = ((link.rx_count - last[1]) / dt, (link.enc_count - last[2]) / dt,
                      (vn.count - last[3]) / dt)
                last = (now, link.rx_count, link.enc_count, vn.count)
            raw = link.enc[1]
            pos = cal.pos(raw) if raw is not None and cal.zero_raw is not None else None
            st = vn.state if vn.available() else {}
            out("tel", t=round(now - t0, 3), raw=raw, pos=pos, deg=None if pos is None else
                cal.deg(pos), sd=None, dir="-", thr=None, brk=None,
                v=None if st.get("speed_mps") is None else round(st["speed_mps"] * 3.6, 2),
                yr=st.get("yaw_rate"), x=st.get("x_m"), y=st.get("y_m"), fix=st.get("fix"),
                rx_hz=round(hz[0], 1), enc_hz=round(hz[1], 1), vn_hz=round(hz[2], 1),
                enc_age=None if link.enc_age() is None else round(link.enc_age(), 2),
                dm=False, need_dm=False, jog=False, fault=link.fault, phase="monitor",
                line=link.last_lines[-1][2][:80] if link.last_lines else "")
            if link.fault:
                print(f"[ERROR] {link.fault}", flush=True)
                return 1
    except KeyboardInterrupt:
        pass
    finally:
        vn.close()
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
