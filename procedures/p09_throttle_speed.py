#!/usr/bin/env python3
"""p09 throttle -> speed, acceleration, coasting and braking (straight line, VectorNav).

Each run: wheels straight, brake off, throttle N until the speed is steady (or a time /
distance limit), then throttle 0 and either coast or brake until stopped.
  throttle_delay   throttle sent -> moving (> 0.3 km/h)
  steady_kmh       mean speed over the last --steady-window s of the throttle phase
  accel_10_90      mean acceleration between 10% and 90% of steady speed (m/s^2)
  accel_max        peak of the smoothed acceleration
  decel            mean deceleration from 80% to 20% of the speed at release (m/s^2)
  stop_distance    distance covered after release (m)
Then: the throttle that gives 15 km/h (interpolated between the values you ran).

Needs a long clear straight: about max-distance metres plus stopping distance.
Writes runs.csv, speed_traces.csv and calibration/longitudinal.json.
"""
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.analysis import clean, linfit  # noqa: E402
from kart.harness import run, run_trial, set_brake, set_throttle, stop_vehicle  # noqa: E402

COLS = ["run", "throttle", "end", "reason", "throttle_delay_s", "steady_kmh", "accel_10_90",
        "accel_max", "release_kmh", "decel", "stop_distance_m", "distance_m"]


def add_args(ap):
    ap.add_argument("--throttle", required=False, default="60,70",
                    help="throttle values to run (comma separated)")
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--then", choices=["coast", "brake", "both"], default="both")
    ap.add_argument("--max-time", type=float, default=15.0, help="longest throttle phase (s)")
    ap.add_argument("--max-distance", type=float, default=60.0, help="metres of clear road")
    ap.add_argument("--steady-window", type=float, default=2.0)
    ap.add_argument("--steady-tol", type=float, default=0.3, help="km/h spread that counts as steady")
    ap.add_argument("--target-kmh", type=float, default=15.0)


def vn_arrays(vn, i0):
    h = [r for r in vn.history[i0:] if r[1] is not None]
    if not h:
        return np.array([]), np.array([]), np.array([]), np.array([])
    t = np.array([r[0] for r in h])
    v = np.array([r[1] for r in h])
    x = np.array([np.nan if r[2] is None else r[2] for r in h])
    y = np.array([np.nan if r[3] is None else r[3] for r in h])
    return t, v, x, y


def body(ctx):
    a, link, steer, cal, rec, vn = ctx.args, ctx.link, ctx.steer, ctx.cal, ctx.rec, ctx.vn
    have_steer = cal.has("steer_map", "zero", "range")
    ends = ["coast", "brake"] if a.then == "both" else [a.then]
    plan = [(int(th), end) for th in a.throttle.split(",") for _ in range(a.reps) for end in ends]
    rec.table("runs", COLS)
    rec.table("speed_traces", ["run", "t_since_throttle", "speed_kmh", "x_m", "y_m"])
    ctx.release_brake_at_end = True
    rows = []
    for k, (th, end) in enumerate(plan, 1):
        ctx.set_phase(f"run {k}/{len(plan)}: throttle {th}, then {end}")
        if not ctx.ch.confirm(f"Run {k}/{len(plan)}: throttle {th}, then {end}. Clear straight "
                              f"for {a.max_distance:.0f} m + stopping, wheels straight, ready?"):
            ctx.log(f"run {k} skipped", "warn")
            continue

        def drive():
            if have_steer:
                steer.move_to(0)
            if link.cmd.brake:
                set_brake(ctx, False)
            i0 = len(vn.history)
            x0, y0 = vn.state.get("x_m") or 0, vn.state.get("y_m") or 0
            t0 = set_throttle(ctx, th)
            reason = "steady"
            while True:
                ctx.sleep(0.05)
                now = time.monotonic()
                dist = math.hypot((vn.state.get("x_m") or 0) - x0, (vn.state.get("y_m") or 0) - y0)
                recent = [r[1] * 3.6 for r in vn.history[i0:] if r[1] is not None
                          and r[0] > now - a.steady_window]
                if now - t0 > a.steady_window + 1 and recent and min(recent) > 1 and \
                        max(recent) - min(recent) < a.steady_tol:
                    break
                if now - t0 > a.max_time:
                    reason = "time limit"
                    break
                if dist > 0.6 * a.max_distance:
                    reason = "distance limit"
                    break
            t_rel = link.send_reliable(throttle=0, brake=1 if end == "brake" else link.cmd.brake,
                                       note=f"release ({end})")
            stop_vehicle(ctx, brake=end == "brake", note=f"release ({end})")
            ctx.sleep(0.5)
            return i0, t0, t_rel, reason

        i0, t0, t_rel, reason = run_trial(ctx, f"run {k}", drive)
        t, v, x, y = vn_arrays(vn, i0)
        for ti, vi, xi, yi in zip(t, v, x, y):
            rec.row("speed_traces", [k, round(ti - t0, 3), round(vi * 3.6, 3), xi, yi])
        acc = t <= t_rel
        steady = v[acc & (t > t_rel - a.steady_window)]
        vs = float(steady.mean()) if len(steady) else float("nan")
        moving = t[(t > t0) & (v > 0.3 / 3.6)]

        def first_t(cond):
            idx = np.where(cond)[0]
            return t[idx[0]] if len(idx) else None
        t10, t90 = first_t(acc & (v >= 0.1 * vs)), first_t(acc & (v >= 0.9 * vs))
        accel = 0.8 * vs / (t90 - t10) if t10 and t90 and t90 > t10 else None
        if acc.sum() > 6:
            vsm = np.convolve(v[acc], np.ones(5) / 5, mode="same")
            accel_max = float(np.max(np.gradient(vsm, t[acc])[2:-2]))
        else:
            accel_max = None
        dec = t > t_rel
        v_rel = v[acc][-1] if acc.sum() else float("nan")
        m = dec & (v <= 0.8 * v_rel) & (v >= 0.2 * v_rel)
        slope = linfit(t[m], v[m])[0] if m.sum() > 3 else None
        trapz = getattr(np, "trapezoid", None) or np.trapz
        stop_dist = float(trapz(v[dec], t[dec])) if dec.sum() > 1 else None
        dist = float(np.nansum(np.hypot(np.diff(x), np.diff(y)))) if len(x) > 1 else None
        row = {"run": k, "throttle": th, "end": end, "reason": reason,
               "throttle_delay_s": float(moving[0] - t0) if len(moving) else None,
               "steady_kmh": vs * 3.6, "accel_10_90": accel, "accel_max": accel_max,
               "release_kmh": v_rel * 3.6, "decel": None if slope is None else -slope,
               "stop_distance_m": stop_dist, "distance_m": dist}
        rec.row("runs", [row[c] for c in COLS])
        rows.append(row)
        ctx.log(f"run {k}: throttle {th} -> {row['steady_kmh']:.1f} km/h ({reason}), accel "
                f"{accel or float('nan'):.2f} m/s2, {end} decel {row['decel'] or float('nan'):.2f} "
                f"m/s2, stop distance {stop_dist or float('nan'):.1f} m", "ok")

    by_th = {}
    for r in rows:
        by_th.setdefault(r["throttle"], []).append(r)
    table = []
    for th, rs in sorted(by_th.items()):
        def mean(key, sel=None):
            vals = [r[key] for r in rs if r[key] is not None and (sel is None or r["end"] == sel)]
            return float(np.mean(vals)) if vals else None
        table.append({"throttle": th, "steady_kmh": mean("steady_kmh"),
                      "accel_10_90": mean("accel_10_90"), "accel_max": mean("accel_max"),
                      "coast_decel": mean("decel", "coast"), "brake_decel": mean("decel", "brake"),
                      "throttle_delay_s": mean("throttle_delay_s")})
    target = None
    pts = [(r["throttle"], r["steady_kmh"]) for r in table if r["steady_kmh"]]
    if len(pts) >= 2:
        ths, sp = zip(*pts)
        a1, b1, _ = linfit(ths, sp)
        if a1:
            target = (a.target_kmh - b1) / a1
    out = clean({"by_throttle": table, f"throttle_for_{a.target_kmh:g}_kmh": target})
    cal.save("longitudinal", out, rec.dir)
    if target:
        ctx.log(f"throttle for {a.target_kmh:g} km/h is about {target:.1f}", "ok")
    return out


if __name__ == "__main__":
    run("09_throttle_speed", "Throttle -> speed, acceleration, coasting, braking", body,
        add_args, needs_vn=True)
