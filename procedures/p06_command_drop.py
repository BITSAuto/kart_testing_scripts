#!/usr/bin/env python3
"""p06 dropped commands: what happens to a command sent soon after another?

Variant 'stop' (default), per trial:
    t0       MOVE (left/right, alternating)
    t0+gap   STOP                        <- the command under test
    t0+gap+rescue  STOP again (rescue), then re-sent until the wheel is still
Classified from the encoder:
    superseded  no movement at all: the STOP replaced the MOVE before it acted
    accepted    movement ended around t0+gap+stop_latency
    dropped     movement ran on until the rescue STOP
Variant 'reverse': MOVE one way then MOVE the other way at t0+gap; accepted = the motion
reversed before the rescue STOP.

Spacings are shuffled. Writes trials.csv and calibration/drop.json.
"""
import random
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.analysis import clean, first_change, linfit, series, settle_time  # noqa: E402
from kart.harness import hold_brake_for_steering, run, run_trial  # noqa: E402

COLS = ["trial", "variant", "direction", "gap_s", "actual_gap_s", "outcome", "motion_start_s",
        "motion_end_s", "travel_ticks"]


def add_args(ap):
    ap.add_argument("--spacings", default="0.05,0.1,0.15,0.2,0.25,0.3,0.4")
    ap.add_argument("--reps", type=int, default=8, help="trials per spacing")
    ap.add_argument("--variant", choices=["stop", "reverse"], default="stop")
    ap.add_argument("--rescue", type=float, default=0.7,
                    help="time after the tested command when the rescue STOP goes out")
    ap.add_argument("--seed", type=int, default=None)


def body(ctx):
    a, link, steer, cal, rec = ctx.args, ctx.link, ctx.steer, ctx.cal, ctx.rec
    rnd = random.Random(a.seed)
    noise = ctx.cfg["encoder"]["noise_ticks"]
    lo, hi = cal.soft_limits()
    lat = cal.get("latency")
    stop_lat = np.mean([lat[d]["stop_latency_s"] or 0.25 for d in ("left", "right")])
    spacings = [float(x) for x in a.spacings.split(",")]
    plan = [g for g in spacings for _ in range(a.reps)]
    rnd.shuffle(plan)
    hold_brake_for_steering(ctx)
    rec.table("trials", COLS)
    rows = []

    def trial(i, gap, d):
        sgn = 1 if d == "left" else -1
        start = 0 if a.variant == "reverse" else -sgn * 0.35 * (hi - lo) / 2
        steer.move_to(start)
        ctx.sleep(1.0)                       # well past any command window
        p_ref = steer.pos()
        t0 = steer.command(d, force=True, note=f"drop test {d}")
        while time.monotonic() < t0 + gap:
            time.sleep(0.0005)
        second = "stop" if a.variant == "stop" else ("right" if d == "left" else "left")
        t1 = steer.command(second, force=True, note=f"drop test 2nd ({gap:.3f})")
        ctx.sleep(max(0.0, t1 + a.rescue - time.monotonic()))
        t2 = steer.command("stop", force=True, note="drop test rescue")
        steer.confirm_stopped(t2)
        t, p = series(link, cal, t0 - 0.1)
        tf, _ = first_change(t, p, t0, p_ref, noise)
        t_end = settle_time(t, p, t0, noise)
        travel = (p[-1] - p_ref) * sgn
        if a.variant == "stop":
            if tf is None or abs(travel) <= 2 * noise:
                outcome = "superseded"
            elif t_end < (t1 + t2) / 2 + stop_lat:
                outcome = "accepted"
            else:
                outcome = "dropped"
        else:
            m = (t > t1 + stop_lat + 0.1) & (t < t2)
            slope, _, _ = linfit(t[m], p[m]) if m.sum() > 3 else (None, None, None)
            if tf is None:
                outcome = "superseded"
            elif slope is not None and slope * sgn < 0:
                outcome = "accepted"
            else:
                outcome = "dropped"
        row = {"trial": i, "variant": a.variant, "direction": d, "gap_s": gap,
               "actual_gap_s": t1 - t0, "outcome": outcome,
               "motion_start_s": None if tf is None else tf - t0,
               "motion_end_s": None if t_end is None else t_end - t0, "travel_ticks": travel}
        rec.row("trials", [row[k] for k in COLS])
        ctx.log(f"{i:3d} gap {gap:.3f} s {d:5s}: {outcome}")
        rows.append(row)

    for i, gap in enumerate(plan, 1):
        d = "left" if i % 2 else "right"
        ctx.set_phase(f"trial {i}/{len(plan)} (gap {gap:.2f} s)")
        run_trial(ctx, f"trial {i}", lambda: trial(i, gap, d))
    run_trial(ctx, "back to centre", lambda: steer.move_to(0))

    table = []
    for g in spacings:
        rs = [r for r in rows if r["gap_s"] == g]
        n = len(rs)
        c = {k: sum(r["outcome"] == k for r in rs) for k in ("accepted", "dropped", "superseded")}
        table.append({"gap_s": g, "n": n, **c,
                      "p_dropped": c["dropped"] / n if n else None,
                      "p_took_effect": (c["accepted"] + c["superseded"]) / n if n else None})
        ctx.log(f"gap {g:.3f} s: accepted {c['accepted']}, superseded {c['superseded']}, "
                f"dropped {c['dropped']} of {n}", "ok")
    safe = [t["gap_s"] for t in table if t["n"] and t["dropped"] == 0]
    out = clean({"variant": a.variant, "by_gap": table,
                 "smallest_gap_without_drops_s": min(safe) if safe else None})
    cal.save("drop", out, rec.dir)
    return out


if __name__ == "__main__":
    run("06_command_drop", "Chance a command is lost vs spacing", body, add_args,
        needs=("steer_map", "zero", "range", "slew", "latency"))
