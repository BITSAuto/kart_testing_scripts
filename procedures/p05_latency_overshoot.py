#!/usr/bin/env python3
"""p05 command delay and overshoot.

Each trial, from standstill: MOVE (left or right, alternating), hold for a random time, STOP.
  dead_time      MOVE sent -> first encoder change (raw, and back-extrapolated along the
                 measured rate to remove the encoder sample interval)
  stop_latency   STOP sent -> wheel settled
  overshoot      travel after STOP was sent (ticks and degrees)
Note: line timestamps are when the line finished arriving; at 9600 baud a 12-character line
takes ~12 ms, so delays include that. The encoder sample interval is reported too.

Writes trials.csv and calibration/latency.json (used by every later move as the stop lead).
"""
import random
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.analysis import clean, first_change, series, settle_time, stats, steady_rate  # noqa
from kart.harness import hold_brake_for_steering, run, run_trial  # noqa: E402

COLS = ["trial", "direction", "hold_s", "t_cmd", "dead_time_s", "dead_time_extrap_s",
        "t_stop_cmd", "stop_latency_s", "overshoot_ticks", "overshoot_deg", "rate_ticks_s",
        "stop_retries"]


def add_args(ap):
    ap.add_argument("--reps", type=int, default=10, help="trials per direction")
    ap.add_argument("--hold-min", type=float, default=0.5)
    ap.add_argument("--hold-max", type=float, default=1.0)
    ap.add_argument("--dwell-min", type=float, default=0.8, help="standstill before each trial")
    ap.add_argument("--dwell-max", type=float, default=1.5)
    ap.add_argument("--seed", type=int, default=None)


def body(ctx):
    a, link, steer, cal, rec = ctx.args, ctx.link, ctx.steer, ctx.cal, ctx.rec
    rnd = random.Random(a.seed)
    noise = ctx.cfg["encoder"]["noise_ticks"]
    lo, hi = cal.soft_limits()
    tpd = cal.ticks_per_deg()
    hold_brake_for_steering(ctx)
    rec.table("trials", COLS)
    rows = []

    def trial(i, d):
        sgn = 1 if d == "left" else -1
        rate = steer.typical_rate(d)
        hold = rnd.uniform(a.hold_min, a.hold_max)
        travel = rate * (hold + 0.3)
        far = hi if d == "left" else lo
        start = far - sgn * min(travel * 1.1, 0.95 * (hi - lo))
        start = min(max(start, lo), hi)
        steer.move_to(start)
        ctx.sleep(rnd.uniform(a.dwell_min, a.dwell_max))
        p_ref = steer.pos()
        n_ev = len(link.samples)
        t_cmd = steer.command(d, force=True, note=f"latency {d}")
        ctx.sleep(hold)
        t_stop = steer.command("stop", force=True, note="latency stop")
        steer.confirm_stopped(t_stop)
        t, p = series(link, cal, t_cmd - 0.2)
        tf, idx = first_change(t, p, t_cmd, p_ref, noise)
        before = p[t <= t_stop]
        p_at_stop = before[-1] if len(before) else p_ref
        m = (t > (tf or t_cmd)) & (t <= t_stop)
        r, _ = steady_rate(t[m], p[m])
        ext = None
        if tf is not None and r:
            ext = max(t_cmd, tf - abs(p[idx] - p_ref) / abs(r)) - t_cmd
        t_set = settle_time(t, p, t_stop, noise)
        over = (p[-1] - p_at_stop) * sgn
        row = {"trial": i, "direction": d, "hold_s": hold, "t_cmd": rec.rel(t_cmd),
               "dead_time_s": None if tf is None else tf - t_cmd, "dead_time_extrap_s": ext,
               "t_stop_cmd": rec.rel(t_stop),
               "stop_latency_s": None if t_set is None else t_set - t_stop,
               "overshoot_ticks": over, "overshoot_deg": over / tpd if tpd else None,
               "rate_ticks_s": r, "stop_retries": 0}
        rec.row("trials", [row[k] for k in COLS])
        ctx.log(f"{i:3d} {d:5s}: delay {row['dead_time_s'] or float('nan'):.3f} s "
                f"(extrap {ext or float('nan'):.3f}), stop {row['stop_latency_s'] or float('nan'):.3f} s, "
                f"overshoot {over:.0f} ticks ({row['overshoot_deg'] or 0:.2f} deg)")
        rows.append(row)
        return n_ev

    i = 0
    for rep in range(a.reps):
        for d in ("left", "right"):
            i += 1
            ctx.set_phase(f"trial {i}/{2 * a.reps}")
            run_trial(ctx, f"trial {i} {d}", lambda: trial(i, d))
    run_trial(ctx, "back to centre", lambda: steer.move_to(0))

    s = link.samples
    dts = np.diff([x[0] for x in s[-2000:]]) if len(s) > 10 else np.array([])
    out = {"encoder_sample_interval_s": stats(dts.tolist()) if len(dts) else None}
    for d in ("left", "right"):
        rs = [r for r in rows if r["direction"] == d]
        dead = stats([r["dead_time_s"] for r in rs])
        ext = stats([r["dead_time_extrap_s"] for r in rs])
        stp = stats([r["stop_latency_s"] for r in rs])
        ov = stats([r["overshoot_ticks"] for r in rs])
        out[d] = {"dead_time_s": dead.get("mean"), "dead_time_p95_s": dead.get("p95"),
                  "dead_time_extrap_s": ext.get("mean"), "stop_latency_s": stp.get("mean"),
                  "stop_latency_p95_s": stp.get("p95"), "overshoot_ticks": ov.get("mean"),
                  "overshoot_deg": ov.get("mean") / tpd if ov.get("mean") is not None and tpd
                  else None,
                  "stats": {"dead_time": dead, "dead_time_extrap": ext, "stop_latency": stp,
                            "overshoot_ticks": ov}}
    out = clean(out)
    cal.save("latency", out, rec.dir)
    for d in ("left", "right"):
        o = out[d]
        ctx.log(f"{d:5s}: delay {o['dead_time_s']} s (p95 {o['dead_time_p95_s']}), stop "
                f"{o['stop_latency_s']} s, overshoot {o['overshoot_deg']} deg", "ok")
    return out


if __name__ == "__main__":
    run("05_latency_overshoot", "Command delay, stop delay, overshoot", body, add_args,
        needs=("steer_map", "zero", "range", "slew"))
