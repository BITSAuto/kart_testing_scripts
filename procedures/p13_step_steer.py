#!/usr/bin/env python3
"""p13 step steer at speed: how the kart's yaw rate answers the steering, while driving.

Each trial: drive straight at --throttle until the speed settles, steer to a fraction of the
soft limit (one side), hold, steer back to straight, hold, stop. Recorded per step:
  wheel angle reached and when, yaw rate (steady, rise 10-90 %, delay from the command to the
  first yaw-rate change), speed, measured curvature yaw/v, and the kinematic bicycle prediction
  v*tan(angle)/L when the wheelbase is known (p14 or [vehicle] wheelbase_m). The ratio
  measured/predicted is the understeer + backlash the twin's tyre model has to reproduce.

Needs VectorNav (yaw rate from $VNYMR / the IMU topic, else from the heading) and an open area:
a step to 60 % lock at 15 km/h turns on a radius of roughly 10 m.
Writes steps.csv, traces.csv and calibration/step_steer.json.
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.analysis import clean, stats  # noqa: E402
from kart.harness import run, run_trial, set_brake, set_throttle, stop_vehicle  # noqa: E402

COLS = ["trial", "side", "fraction", "speed_kmh", "target_deg", "reached_deg", "t_reach_s",
        "yaw_delay_s", "yaw_rise_s", "yaw_rate_ss", "curvature_meas", "curvature_bicycle",
        "ratio_meas_over_bicycle"]


def add_args(ap):
    ap.add_argument("--throttle", type=int, required=False, default=None,
                    help="throttle (default: the 15 km/h value from p09)")
    ap.add_argument("--fractions", default="0.3,0.6", help="step sizes, fractions of the soft limit")
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--settle", type=float, default=5.0, help="straight driving before the step (s)")
    ap.add_argument("--hold", type=float, default=2.5, help="seconds held at the step angle")


def yaw_series(vn, i0):
    h = vn.history[i0:]
    t = np.array([r[0] for r in h])
    yr = np.array([np.nan if r[5] is None else r[5] for r in h])
    if np.isnan(yr).all():                       # no gyro: differentiate the heading
        yaw = np.array([np.nan if r[4] is None else r[4] for r in h])
        m = ~np.isnan(yaw)
        yr = np.full(len(t), np.nan)
        if m.sum() > 3:
            # VectorNav heading is clockwise from north: left turn = heading decreasing
            yr[m] = -np.gradient(np.unwrap(np.radians(yaw[m])), t[m])
    v = np.array([np.nan if r[1] is None else r[1] for r in h])
    return t, yr, v


def body(ctx):
    a, link, steer, cal, rec, vn = ctx.args, ctx.link, ctx.steer, ctx.cal, ctx.rec, ctx.vn
    th = a.throttle
    if th is None:
        lo_cal = cal.get("longitudinal") or {}
        th = next((v for k, v in lo_cal.items() if k.startswith("throttle_for_")), None)
        if th is None:
            raise RuntimeError("give --throttle (no p09 result to take it from)")
        th = int(round(th))
    veh = cal.get("vehicle") or {}
    L = veh.get("wheelbase_m") or ctx.cfg.get("vehicle", {}).get("wheelbase_m") or None
    lo, hi = cal.soft_limits()
    rec.table("steps", COLS)
    rec.table("traces", ["trial", "t_since_step", "pos", "deg", "yaw_rate", "speed_kmh"])
    ctx.release_brake_at_end = True
    plan = [(f, s) for _ in range(a.reps) for f in [float(x) for x in a.fractions.split(",")]
            for s in ("left", "right")]
    rows = []
    for k, (frac, side) in enumerate(plan, 1):
        target = frac * (hi if side == "left" else lo)
        tdeg = cal.deg(target)
        ctx.set_phase(f"step {k}/{len(plan)}: {side} {frac:.0%}")
        if not ctx.ch.confirm(f"Step {k}/{len(plan)}: drive straight at throttle {th}, then steer "
                              f"{side} to {tdeg:+.1f} deg for {a.hold:.1f} s. Area clear?"):
            continue

        def drive():
            steer.move_to(0)
            if link.cmd.brake:
                set_brake(ctx, False)
            set_throttle(ctx, th)
            ctx.sleep(a.settle)
            i0 = len(vn.history)
            n0 = len(link.samples)
            t_step = time.monotonic()
            steer.move_to(target)
            t_reach = time.monotonic()
            ctx.sleep(a.hold)
            t_hold_end = time.monotonic()
            steer.move_to(0)
            ctx.sleep(1.0)
            link.send_reliable(throttle=0, brake=1, note="step end")
            stop_vehicle(ctx, note="step end")
            return i0, n0, t_step, t_reach, t_hold_end

        i0, n0, t_step, t_reach, t_hold_end = run_trial(ctx, f"step {k}", drive)
        t, yr, v = yaw_series(vn, i0 - 40)
        s = link.samples[n0:]
        for ti, y, vi in zip(t, yr, v):
            p = None
            for ts, raw in s:
                if ts > ti:
                    break
                p = cal.pos(raw)
            rec.row("traces", [k, round(ti - t_step, 3), p, None if p is None else cal.deg(p),
                               None if np.isnan(y) else round(float(y), 5),
                               None if np.isnan(vi) else round(float(vi) * 3.6, 3)])
        pre = (t < t_step) & ~np.isnan(yr)
        base = float(np.nanmean(yr[pre])) if pre.any() else 0.0
        ss_m = (t > t_hold_end - 1.0) & (t <= t_hold_end) & ~np.isnan(yr)
        yss = float(np.nanmean(yr[ss_m])) - base if ss_m.any() else None
        vss = float(np.nanmean(v[ss_m])) if ss_m.any() else None
        delay = rise = None
        if yss:
            d = (yr - base) * np.sign(yss)
            after = (t > t_step) & (t <= t_hold_end)
            i_start = np.where(after & (d > 0.1 * abs(yss)))[0]
            i_90 = np.where(after & (d > 0.9 * abs(yss)))[0]
            if len(i_start):
                delay = float(t[i_start[0]] - t_step)
            if len(i_start) and len(i_90):
                rise = float(t[i_90[0]] - t[i_start[0]])
        reached = steer.cal.deg(cal.pos(s[-1][1])) if s else None
        hold_pos = [cal.pos(r) for ts, r in s if t_reach <= ts <= t_hold_end]
        reached = cal.deg(float(np.median(hold_pos))) if hold_pos else reached
        kappa = yss / vss if yss is not None and vss else None
        kb = np.tan(np.radians(reached)) / L if L and reached is not None else None
        row = {"trial": k, "side": side, "fraction": frac,
               "speed_kmh": None if vss is None else vss * 3.6, "target_deg": tdeg,
               "reached_deg": reached, "t_reach_s": t_reach - t_step, "yaw_delay_s": delay,
               "yaw_rise_s": rise, "yaw_rate_ss": yss, "curvature_meas": kappa,
               "curvature_bicycle": kb,
               "ratio_meas_over_bicycle": kappa / kb if kappa and kb else None}
        rec.row("steps", [row[c] for c in COLS])
        rows.append(row)
        ctx.log(f"step {k} {side} {frac:.0%}: wheel {reached if reached is None else round(reached, 2)} "
                f"deg in {row['t_reach_s']:.2f} s, yaw {yss if yss is None else round(yss, 3)} rad/s "
                f"(delay {delay if delay is None else round(delay, 2)} s, rise "
                f"{rise if rise is None else round(rise, 2)} s), curvature meas/bicycle "
                f"{row['ratio_meas_over_bicycle'] if row['ratio_meas_over_bicycle'] is None else round(row['ratio_meas_over_bicycle'], 3)}",
                "ok")
    out = clean({"throttle": th, "wheelbase_m": L, "steps": rows,
                 "yaw_delay_s": stats([r["yaw_delay_s"] for r in rows]),
                 "yaw_rise_s": stats([r["yaw_rise_s"] for r in rows]),
                 "ratio_meas_over_bicycle": stats([r["ratio_meas_over_bicycle"] for r in rows])})
    if rows:
        cal.save("step_steer", out, rec.dir)
    return out


if __name__ == "__main__":
    run("13_step_steer", "Yaw-rate response to steering steps at speed", body, add_args,
        needs=("steer_map", "zero", "range"), needs_vn=True)
