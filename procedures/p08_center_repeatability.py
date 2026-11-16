#!/usr/bin/env python3
"""p08 centre repeatability / encoder drift.

Random excursions left and right, back to encoder zero each time, and you type the road-wheel
angle. If the measured angle at "zero" creeps in one direction over the trials, the encoder
is slipping or missing counts; the spread shows how repeatable centre is (backlash included).

Writes trials.csv and calibration/repeatability.json.
"""
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.analysis import clean, linfit, stats  # noqa: E402
from kart.harness import hold_brake_for_steering, run, run_trial  # noqa: E402


def add_args(ap):
    ap.add_argument("--trials", type=int, default=6)
    ap.add_argument("--min-frac", type=float, default=0.3)
    ap.add_argument("--max-frac", type=float, default=0.9)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--no-ask", action="store_true", help="don't ask for angles (encoder only)")


def body(ctx):
    a, steer, cal, rec = ctx.args, ctx.steer, ctx.cal, ctx.rec
    rnd = random.Random(a.seed)
    lo, hi = cal.soft_limits()
    hold_brake_for_steering(ctx)
    rec.table("trials", ["trial", "excursion_pos", "final_pos", "final_encoder_deg",
                         "measured_deg"])
    rows = []
    for i in range(1, a.trials + 1):
        side = rnd.choice((-1, 1))
        exc = rnd.uniform(a.min_frac, a.max_frac) * (hi if side > 0 else -lo) * side
        ctx.set_phase(f"trial {i}/{a.trials}")

        def go():
            steer.move_to(exc)
            ctx.sleep(0.5)
            steer.move_to(0, tol=max(steer.default_tol() / 2, 2))
            return steer.wait_quiet()
        final = run_trial(ctx, f"trial {i}", go)
        meas = None if a.no_ask else ctx.ch.ask(
            f"Trial {i}: back at encoder zero (pos {final}). Road-wheel angle in degrees (+ = left)",
            "number")
        r = [i, exc, final, cal.deg(final), meas]
        rec.row("trials", r)
        rows.append(r)
    meas = [r[4] for r in rows if r[4] is not None]
    slope = linfit(list(range(len(meas))), meas)[0] if len(meas) > 2 else None
    out = clean({"final_pos": stats([r[2] for r in rows]), "measured_deg": stats(meas),
                 "drift_deg_per_trial": slope})
    cal.save("repeatability", out, rec.dir)
    ctx.log(f"angle at zero: mean {out['measured_deg'].get('mean')} deg, spread (std) "
            f"{out['measured_deg'].get('std')} deg, drift {slope} deg/trial", "ok")
    return out


if __name__ == "__main__":
    run("08_center_repeatability", "Return-to-centre repeatability and encoder drift", body,
        add_args, needs=("steer_map", "zero", "range"))
