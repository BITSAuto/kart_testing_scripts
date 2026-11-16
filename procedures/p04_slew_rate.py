#!/usr/bin/env python3
"""p04 steering rate (slew): how fast does the wheel turn, each way, across the range?

Each rep: go to the right end of the span, sweep to the left end, sweep back. From every
sweep: delay to first movement, steady rate (line fit over the middle 60% of the travel),
and the local rate at each position (does it slow down near the locks / under load?).

--throttle N repeats it rolling (needs space: the kart drives arcs). Results go to
calibration/slew.json (standstill) or slew_rolling.json, plus sweeps.csv and rate_profile.csv.
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.analysis import clean, first_change, series, stats, steady_rate  # noqa: E402
from kart.harness import hold_brake_for_steering, run, run_trial, set_throttle  # noqa: E402


def add_args(ap):
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--span", type=float, default=0.8, help="fraction of the soft range to sweep")
    ap.add_argument("--throttle", type=int, default=0, help="throttle while sweeping (0 = parked)")
    ap.add_argument("--bins", type=int, default=10, help="position bins for the rate profile")


def body(ctx):
    a, link, steer, cal, rec, guard = ctx.args, ctx.link, ctx.steer, ctx.cal, ctx.rec, ctx.guard
    noise = ctx.cfg["encoder"]["noise_ticks"]
    lo, hi = cal.soft_limits()
    tpd = cal.ticks_per_deg()
    left_t, right_t = a.span * hi, a.span * lo
    rolling = a.throttle > 0
    if not rolling:
        hold_brake_for_steering(ctx)
    rec.table("sweeps", ["rep", "direction", "t_cmd", "dead_time_s", "rate_ticks_s", "rate_deg_s",
                         "r2", "start_pos", "end_pos", "duration_s", "speed_kmh"])
    sweeps, local = [], {"left": [], "right": []}

    def sweep(rep, d):
        target = left_t if d == "left" else right_t
        sgn = 1 if d == "left" else -1
        p0 = steer.pos()
        t_cmd = steer.command(d, note=f"sweep {d}")
        while True:
            ctx.sleep(0.005)
            if (target - steer.pos()) * sgn <= steer.stop_lead_ticks(d) or guard.limit_hit == d:
                break
            if time.monotonic() - t_cmd > guard.max_move_s():
                break
        t_stop = steer.stop(note="sweep stop")
        t, p = series(link, cal, t_cmd - 0.05)
        tf, _ = first_change(t, p, t_cmd, p0, noise)
        m = (t >= (tf or t_cmd)) & (t <= t_stop)
        rate, r2 = steady_rate(t[m], p[m])
        tm, pm = t[m], p[m]
        for i in range(2, len(tm) - 2):         # local rate over +-2 samples
            dt = tm[i + 2] - tm[i - 2]
            if dt > 0:
                local[d].append((pm[i], (pm[i + 2] - pm[i - 2]) / dt))
        v = ctx.vn.speed_kmh()
        row = {"rep": rep, "direction": d, "t_cmd": rec.rel(t_cmd),
               "dead_time_s": None if tf is None else tf - t_cmd, "rate_ticks_s": rate,
               "rate_deg_s": None if rate is None or not tpd else rate / tpd, "r2": r2,
               "start_pos": p0, "end_pos": steer.pos(), "duration_s": t_stop - t_cmd,
               "speed_kmh": v}
        rec.row("sweeps", [row[k] for k in ("rep", "direction", "t_cmd", "dead_time_s",
                                            "rate_ticks_s", "rate_deg_s", "r2", "start_pos",
                                            "end_pos", "duration_s", "speed_kmh")])
        ctx.log(f"rep {rep} {d:5s}: rate {abs(rate or 0):.0f} ticks/s"
                f" ({abs(row['rate_deg_s'] or 0):.2f} deg/s), delay "
                f"{row['dead_time_s'] if row['dead_time_s'] is None else round(row['dead_time_s'], 3)} s")
        sweeps.append(row)

    def rep_block(rep):
        if rolling:
            set_throttle(ctx, a.throttle)
            ctx.sleep(2.0)
        steer.move_to(right_t)
        sweep(rep, "left")
        sweep(rep, "right")

    for rep in range(1, a.reps + 1):
        ctx.set_phase(f"rep {rep}/{a.reps}")
        run_trial(ctx, f"rep {rep}", lambda: rep_block(rep))
    run_trial(ctx, "back to centre", lambda: steer.move_to(0))

    out = {"condition": "rolling" if rolling else "standstill", "throttle": a.throttle,
           "span": a.span}
    for d in ("left", "right"):
        rs = [s for s in sweeps if s["direction"] == d and s["rate_ticks_s"] is not None]
        st = stats([abs(s["rate_ticks_s"]) for s in rs])
        out[d] = {"rate_ticks_s": st.get("mean"), "rate_ticks_s_stats": st,
                  "rate_deg_s": st.get("mean") / tpd if st.get("mean") and tpd else None,
                  "dead_time_s": stats([s["dead_time_s"] for s in rs])}
        # profile
        pts = np.array(local[d]) if local[d] else np.zeros((0, 2))
        prof = []
        if len(pts):
            edges = np.linspace(lo, hi, a.bins + 1)
            for i in range(a.bins):
                m = (pts[:, 0] >= edges[i]) & (pts[:, 0] < edges[i + 1])
                if m.sum() >= 3:
                    c = (edges[i] + edges[i + 1]) / 2
                    prof.append({"pos": c, "deg": cal.deg(c), "rate_ticks_s":
                                 float(np.median(np.abs(pts[m, 1]))), "n": int(m.sum())})
        out[d]["profile"] = prof
    rec.table("rate_profile", ["direction", "pos", "deg", "rate_ticks_s", "n"])
    for d in ("left", "right"):
        for r in out[d]["profile"]:
            rec.row("rate_profile", [d, r["pos"], r["deg"], r["rate_ticks_s"], r["n"]])
    if out["left"]["rate_ticks_s"] and out["right"]["rate_ticks_s"]:
        out["left_over_right"] = out["left"]["rate_ticks_s"] / out["right"]["rate_ticks_s"]
    out = clean(out)
    cal.save("slew_rolling" if rolling else "slew", out, rec.dir)
    if out.get("left_over_right"):
        ctx.log(f"left {out['left']['rate_deg_s']:.2f} deg/s, right "
                f"{out['right']['rate_deg_s']:.2f} deg/s (left/right = "
                f"{out['left_over_right']:.2f})", "ok")
    return out


if __name__ == "__main__":
    run("04_slew_rate", "Steering rate each way and across the range", body, add_args,
        needs=("steer_map", "zero", "range"), needs_vn=lambda a: a.throttle > 0)
