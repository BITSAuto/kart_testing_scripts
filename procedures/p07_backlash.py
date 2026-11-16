#!/usr/bin/env python3
"""p07 backlash: play between the steering wheel (where the encoder is) and the road wheels.

At each position the kart stops at the same encoder count twice: once arriving from the left
(moving right), once from the right (moving left). You type the road-wheel angle each time.
Backlash = difference in measured angle, after correcting for the small difference in
where the encoder actually stopped.

Measure the same wheel every time, as precisely as you can (0.1 deg: phone inclinometer on a
straight edge, or a laser on a wall a few metres away). Writes readings.csv and
calibration/backlash.json.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.analysis import clean, stats  # noqa: E402
from kart.harness import hold_brake_for_steering, run, run_trial  # noqa: E402


def add_args(ap):
    ap.add_argument("--positions", default="0,0.5,-0.5", help="fractions of the soft range")
    ap.add_argument("--approach", type=float, default=0.25,
                    help="how far away to start each approach (fraction of the range)")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--wheel", choices=["left", "right"], default="left",
                    help="which front wheel you measure")


def body(ctx):
    a, link, steer, cal, rec = ctx.args, ctx.link, ctx.steer, ctx.cal, ctx.rec
    lo, hi = cal.soft_limits()
    tpd = cal.ticks_per_deg()
    hold_brake_for_steering(ctx)
    rec.table("readings", ["position", "rep", "from_side", "pos", "encoder_deg", "measured_deg"])
    readings = []
    span = (hi - lo) / 2
    tol = max(steer.default_tol() / 2, 2)
    for frac in [float(x) for x in a.positions.split(",")]:
        target = frac * (hi if frac >= 0 else -lo)
        for rep in range(1, a.reps + 1):
            for side in ("left", "right"):
                start = target + (a.approach * span if side == "left" else -a.approach * span)
                start = min(max(start, lo), hi)
                ctx.set_phase(f"pos {frac:+.2f}, rep {rep}, from the {side}")

                def go():
                    steer.move_to(start)
                    steer.move_to(target, tol=tol)
                    return steer.wait_quiet()
                pos = run_trial(ctx, f"{frac:+.2f} from {side}", go)
                deg = ctx.ch.ask(f"Encoder pos {pos} (approached from the {side}). {a.wheel.upper()} "
                                 "front wheel angle in degrees (+ = left)", "number")
                r = {"position": frac, "rep": rep, "from_side": side, "pos": pos,
                     "encoder_deg": cal.deg(pos), "measured_deg": deg}
                rec.row("readings", list(r.values()))
                readings.append(r)
    run_trial(ctx, "back to centre", lambda: steer.move_to(0))

    per = []
    for frac in sorted({r["position"] for r in readings}):
        vals = []
        for rep in range(1, a.reps + 1):
            L = next((r for r in readings if r["position"] == frac and r["rep"] == rep
                      and r["from_side"] == "left"), None)
            R = next((r for r in readings if r["position"] == frac and r["rep"] == rep
                      and r["from_side"] == "right"), None)
            if L and R:
                enc_diff = (L["pos"] - R["pos"]) / tpd
                vals.append((L["measured_deg"] - R["measured_deg"]) - enc_diff)
        st = stats(vals)
        per.append({"position": frac, "backlash_deg": st.get("mean"), "stats": st})
        ctx.log(f"position {frac:+.2f}: backlash {st.get('mean')} deg (n={st.get('n')})", "ok")
    allv = [p["backlash_deg"] for p in per if p["backlash_deg"] is not None]
    out = clean({"backlash_deg": stats(allv).get("mean"), "by_position": per, "wheel": a.wheel,
                 "note": "positive = road wheel lags the steering wheel"})
    cal.save("backlash", out, rec.dir)
    return out


if __name__ == "__main__":
    run("07_backlash", "Play between steering wheel and road wheels", body, add_args,
        needs=("steer_map", "zero", "range"))
