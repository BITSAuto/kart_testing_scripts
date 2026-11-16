#!/usr/bin/env python3
"""p10 full-lock circles: the real turning radius.

For each side: steer to the soft limit at standstill, drive at --throttle until the heading
has turned --turns times around (or --max-time), release and brake. A circle is fitted to the
VectorNav track from the steady part; radius also from speed / yaw rate if available.
With [vehicle] wheelbase_m set, gives the effective steering angle atan(L/R) to compare with
the angle table from p03 (the difference is tyre slip + Ackermann + backlash).

Space: a circle of about 2 x radius (roughly 6-12 m) plus margin, both ways.
Writes circles.csv, tracks.csv and calibration/circles.json.
"""
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.analysis import circle_fit, clean  # noqa: E402
from kart.harness import run, run_trial, set_brake, set_throttle, stop_vehicle  # noqa: E402


def add_args(ap):
    ap.add_argument("--throttle", type=int, required=False, default=60)
    ap.add_argument("--sides", default="left,right")
    ap.add_argument("--fraction", type=float, default=1.0, help="fraction of the soft limit")
    ap.add_argument("--turns", type=float, default=1.25, help="laps per side")
    ap.add_argument("--max-time", type=float, default=22.0)
    ap.add_argument("--settle", type=float, default=3.0, help="seconds ignored at the start")


def body(ctx):
    a, link, steer, cal, rec, vn = ctx.args, ctx.link, ctx.steer, ctx.cal, ctx.rec, ctx.vn
    lo, hi = cal.soft_limits()
    L = (cal.get("vehicle") or {}).get("wheelbase_m") or ctx.cfg.get("vehicle", {}).get("wheelbase_m") or 0
    rec.table("circles", ["side", "pos", "table_deg", "radius_fit_m", "fit_rms_m", "radius_v_over_r_m",
                          "speed_kmh", "effective_deg"])
    rec.table("tracks", ["side", "t", "x_m", "y_m", "speed_kmh", "yaw_deg", "yaw_rate"])
    ctx.release_brake_at_end = True
    results = []
    for side in a.sides.split(","):
        side = side.strip()
        target = a.fraction * (hi if side == "left" else lo)
        ctx.set_phase(f"{side} circle")
        if not ctx.ch.confirm(f"{side.upper()} circle at throttle {a.throttle}: clear area of about "
                              "15 m across, ready?"):
            continue

        def drive():
            steer.move_to(target)
            if link.cmd.brake:
                set_brake(ctx, False)
            i0 = len(vn.history)
            t0 = set_throttle(ctx, a.throttle)
            turned, last_yaw = 0.0, vn.state.get("yaw_deg")
            while True:
                ctx.sleep(0.05)
                yaw = vn.state.get("yaw_deg")
                if yaw is not None and last_yaw is not None:
                    turned += abs((yaw - last_yaw + 180) % 360 - 180)
                last_yaw = yaw
                if turned >= 360 * a.turns or time.monotonic() - t0 > a.max_time:
                    break
            t_rel = link.send_reliable(throttle=0, brake=1, note="circle end")
            stop_vehicle(ctx, note="circle end")
            return i0, t0, t_rel, turned

        i0, t0, t_rel, turned = run_trial(ctx, f"{side} circle", drive)
        h = [r for r in vn.history[i0:] if t0 + a.settle < r[0] < t_rel and r[2] is not None]
        for r in vn.history[i0:]:
            rec.row("tracks", [side, round(r[0] - t0, 3), r[2], r[3],
                               None if r[1] is None else r[1] * 3.6, r[4], r[5]])
        fit = circle_fit([r[2] for r in h], [r[3] for r in h]) if len(h) > 10 else None
        sp = np.array([r[1] for r in h if r[1] is not None])
        yr = np.array([r[5] for r in h if r[5] is not None])
        r_vr = float(np.mean(sp) / abs(np.mean(yr))) if len(sp) and len(yr) and abs(np.mean(yr)) > 1e-3 else None
        radius = fit[2] if fit else r_vr
        eff = math.degrees(math.atan(L / radius)) * (1 if side == "left" else -1) \
            if L and radius else None
        pos = steer.pos()
        res = {"side": side, "pos": pos, "table_deg": cal.deg(pos),
               "radius_fit_m": fit[2] if fit else None, "fit_rms_m": fit[3] if fit else None,
               "radius_v_over_r_m": r_vr, "speed_kmh": float(np.mean(sp) * 3.6) if len(sp) else None,
               "effective_deg": eff, "turned_deg": turned}
        rec.row("circles", [res[k] for k in ("side", "pos", "table_deg", "radius_fit_m", "fit_rms_m",
                                             "radius_v_over_r_m", "speed_kmh", "effective_deg")])
        results.append(res)
        ctx.log(f"{side}: radius {res['radius_fit_m']} m (fit rms {res['fit_rms_m']}), v/yaw-rate "
                f"{r_vr} m, table angle {res['table_deg']} deg, effective {eff} deg", "ok")
        run_trial(ctx, "wheels straight", lambda: steer.move_to(0))
    out = clean({"throttle": a.throttle, "wheelbase_m": L or None, "circles": results})
    if results:
        cal.save("circles", out, rec.dir)
    return out


if __name__ == "__main__":
    run("10_lock_circles", "Turning radius at full lock", body, add_args,
        needs=("steer_map", "zero", "range"), needs_vn=True)
