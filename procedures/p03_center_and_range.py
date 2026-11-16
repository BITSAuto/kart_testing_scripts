#!/usr/bin/env python3
"""p03 centre, locks and angle table.

1. You jog the wheels straight ahead -> encoder zero (data/calibration/zero.json).
   The encoder is incremental: redo this (--zero-only) after every ESP32 power cycle.
2. You jog to full LEFT lock, type the road-wheel angle, then the same for RIGHT.
3. The kart stops at several positions in between; you type the road-wheel angle each time.
   -> data/calibration/range.json: lock positions, ticks per degree, the full table, linearity.

Measuring: a phone inclinometer on a straight edge against the tyre, or a laser pointer
taped to the wheel shining on a floor protractor. + = left. With --both-wheels you also type
the right wheel (inner/outer angles differ: Ackermann, needed for the digital twin).
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.analysis import clean, linfit  # noqa: E402
from kart.harness import hold_brake_for_steering, jog_session, run, run_trial  # noqa: E402


def add_args(ap):
    ap.add_argument("--zero-only", action="store_true", help="only set the straight-ahead zero")
    ap.add_argument("--points", type=int, default=7,
                    help="positions in the angle table, including both locks and centre")
    ap.add_argument("--both-wheels", action="store_true", help="also ask for the right wheel")


def ask_angles(ctx, where, both):
    left = ctx.ch.ask(f"{where}: LEFT front wheel angle in degrees (+ = pointing left)", "number")
    right = ctx.ch.ask(f"{where}: RIGHT front wheel angle in degrees (+ = pointing left)",
                       "number") if both else None
    return left, right


def body(ctx):
    a, link, steer, cal, rec = ctx.args, ctx.link, ctx.steer, ctx.cal, ctx.rec
    hold_brake_for_steering(ctx)
    rec.table("angle_table", ["pos", "raw", "left_wheel_deg", "right_wheel_deg", "deg"])

    ctx.set_phase("zero: jog straight ahead")
    jog_session(ctx, "Steer the wheels STRAIGHT AHEAD (line them up with your centre mark)")
    steer.wait_quiet()
    raw0 = link.enc[1]
    cal.save("zero", {"raw_ticks": raw0}, rec.dir)
    ctx.log(f"zero set: raw {raw0}", "ok")
    if a.zero_only:
        return {"zero_raw": raw0}
    center_l, center_r = ask_angles(ctx, "Straight ahead", a.both_wheels)
    rows = [(0, raw0, center_l, center_r)]

    ctx.set_phase("left lock")
    jog_session(ctx, "Steer slowly to full LEFT lock and stop as it reaches the end "
                     "(don't keep pushing against it)")
    steer.wait_quiet()
    left_pos, raw = cal.pos(link.enc[1]), link.enc[1]
    rows.append((left_pos, raw, *ask_angles(ctx, "LEFT lock", a.both_wheels)))

    ctx.set_phase("right lock")
    jog_session(ctx, "Now steer slowly to full RIGHT lock and stop at the end")
    steer.wait_quiet()
    right_pos, raw = cal.pos(link.enc[1]), link.enc[1]
    rows.append((right_pos, raw, *ask_angles(ctx, "RIGHT lock", a.both_wheels)))
    if not (left_pos > 0 > right_pos):
        raise RuntimeError(f"left lock at pos {left_pos}, right lock at {right_pos}: left should be "
                           "positive. Rerun p02 (the encoder sign is wrong) or redo the zero.")

    # provisional range (no angle table yet) so moves are allowed and limited to the locks
    cal.save("range", {"lock_left_pos": left_pos, "lock_right_pos": right_pos,
                       "deg_table": [], "ticks_per_deg": None, "provisional": True}, rec.dir)

    n_inner = max(a.points - 3, 0)
    fracs = list(np.linspace(-1, 1, n_inner + 2)[1:-1]) if n_inner else []
    fracs = [0.9 * f for f in fracs]  # keep the inner points clear of the locks
    for f in fracs:
        target = f * left_pos if f > 0 else -f * right_pos
        ctx.set_phase(f"angle table: {f:+.0%} of lock")
        run_trial(ctx, f"move to {target:.0f}", lambda: steer.move_to(target))
        steer.wait_quiet()
        pos = cal.pos(link.enc[1])
        rows.append((pos, link.enc[1], *ask_angles(ctx, f"Position {pos}", a.both_wheels)))
    run_trial(ctx, "back to centre", lambda: steer.move_to(0))

    table = []
    for pos, raw, l_deg, r_deg in sorted(rows):
        deg = l_deg if r_deg is None else (l_deg + r_deg) / 2
        table.append([pos, deg])
        rec.row("angle_table", [pos, raw, l_deg, r_deg, deg])
    pos_arr, deg_arr = np.array(table).T
    slope, icpt, r2 = linfit(pos_arr, deg_arr)
    resid = deg_arr - (slope * pos_arr + icpt)
    out = clean({
        "lock_left_pos": left_pos, "lock_right_pos": right_pos,
        "lock_left_deg": float(deg_arr[pos_arr.argmax()]),
        "lock_right_deg": float(deg_arr[pos_arr.argmin()]),
        "ticks_per_deg": 1 / slope, "deg_offset_at_zero": icpt, "linear_r2": r2,
        "max_residual_deg": float(np.abs(resid).max()),
        "deg_table": table,
        "per_wheel": [{"pos": p, "left_deg": l, "right_deg": r} for p, _, l, r in sorted(rows)],
        "zero_raw": raw0, "provisional": False,
    })
    cal.save("range", out, rec.dir)
    ctx.log(f"locks: left {out['lock_left_deg']:+.1f} deg ({left_pos} ticks), right "
            f"{out['lock_right_deg']:+.1f} deg ({right_pos} ticks); {out['ticks_per_deg']:.1f} "
            f"ticks/deg, linear fit r2 {r2:.4f}, worst residual {out['max_residual_deg']:.2f} deg",
            "ok")
    return out


if __name__ == "__main__":
    run("03_center_and_range", "Straight-ahead zero, lock positions, ticks -> wheel angle",
        body, add_args, needs=("steer_map",),
        motion=True)
