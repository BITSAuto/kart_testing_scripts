#!/usr/bin/env python3
"""p02 steering command map: what do the D (steering) values actually do?

Short pulses at each D value, alternating sides so the wheels stay near where they started:
  * which side of 50 increases the encoder count, and the deadband around 50
  * steering rate (encoder ticks/s) for each value: constant or proportional to |D-50|?
  * delay from command to first movement for each value
Then one pulse you watch, to confirm by eye which side is LEFT.

Writes data/calibration/steer_map.json (left/right D values, encoder sign) and pulses.csv.
Kart at standstill (brake held) or front wheels off the ground.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.analysis import clean, first_change, steady_rate  # noqa: E402
from kart.harness import hold_brake_for_steering, run, run_trial  # noqa: E402

import numpy as np  # noqa: E402


def add_args(ap):
    ap.add_argument("--values", default="40,30,20,0,60,70,80,100",
                    help="D values to try (comma separated)")
    ap.add_argument("--pulse-s", type=float, default=0.4, help="pulse length (s)")
    ap.add_argument("--confirm-s", type=float, default=0.5, help="length of the pulse you watch")


def raw_series(link, t0):
    s = link.samples_between(t0)
    return np.array([a for a, _ in s]), np.array([b for _, b in s], float)


def body(ctx):
    a, link, steer, cal, rec = ctx.args, ctx.link, ctx.steer, ctx.cal, ctx.rec
    stop = cal.stop_value
    noise = ctx.cfg["encoder"]["noise_ticks"]
    pulse_s = min(a.pulse_s, ctx.cfg["safety"]["uncalibrated_pulse_s"])
    values = [int(v) for v in a.values.split(",") if v.strip()]
    lows = sorted([v for v in values if v < stop], reverse=True)
    highs = sorted([v for v in values if v > stop])
    order = [v for pair in zip(lows, highs) for v in pair] + lows[len(highs):] + highs[len(lows):]
    rec.table("pulses", ["value", "raw_start", "raw_end", "moved_raw", "rate_raw_s",
                         "dead_time_s", "t_cmd"])
    hold_brake_for_steering(ctx)
    if link.enc[1] is None:
        raise RuntimeError("no encoder data")
    start_raw = link.enc[1]
    results = []
    up_value = down_value = None     # D values seen to increase / decrease the raw count

    def pulse(v, dur):
        r0 = link.enc[1]
        t0 = steer.raw_command(v, note=f"map pulse D={v}")
        ctx.sleep(dur)
        t1 = steer.stop(note="map pulse stop")
        t, r = raw_series(link, t0 - 0.05)
        tf, _ = first_change(t, r, t0, r0, noise)
        m = (t >= t0) & (t <= t1 + 0.05)
        rate, _ = steady_rate(t[m], r[m]) if m.sum() > 3 else (None, None)
        return {"value": v, "raw_start": r0, "raw_end": link.enc[1], "moved_raw": link.enc[1] - r0,
                "rate_raw_s": rate, "dead_time_s": None if tf is None else tf - t0, "t_cmd": t0}

    def go_back(target_raw):
        """Return near target_raw with short pulses, using D values seen to move each way."""
        for _ in range(8):
            err = target_raw - link.enc[1]
            v = up_value if err > 0 else down_value
            if v is None or abs(err) <= 3 * noise:
                return
            rate = next((abs(r["rate_raw_s"]) for r in results
                         if r["value"] == v and r["rate_raw_s"]), None)
            dur = pulse_s if not rate else min(pulse_s, max(0.25, abs(err) / rate - 0.15))
            run_trial(ctx, f"return D={v}", lambda: pulse(v, dur))

    ctx.ch.ask("Wheels roughly straight, kart braked or front wheels lifted? Press Enter to start",
               default="")
    for v in order:
        ctx.set_phase(f"pulse D={v}")
        r = run_trial(ctx, f"pulse D={v}", lambda: pulse(v, pulse_s))
        if abs(r["moved_raw"]) <= 2 * noise and 0 <= 2 * stop - v <= 100:
            # No movement: maybe sitting at a lock, or the command was lost. Nudge the other
            # way once and try again before calling it deadband.
            ctx.log(f"D={v} did not move; nudging with D={2 * stop - v} and retrying", "warn")
            run_trial(ctx, f"nudge D={2 * stop - v}", lambda: pulse(2 * stop - v, pulse_s))
            r = run_trial(ctx, f"retry D={v}", lambda: pulse(v, pulse_s))
            r["retried"] = True
        results.append(r)
        rec.row("pulses", [r[k] for k in ("value", "raw_start", "raw_end", "moved_raw",
                                          "rate_raw_s", "dead_time_s")] + [rec.rel(r["t_cmd"])])
        rate_txt = "-" if r["rate_raw_s"] is None else f"{r['rate_raw_s']:+.0f}"
        ctx.log(f"D={v:3d}: moved {r['moved_raw']:+5d} ticks, rate {rate_txt} ticks/s, "
                f"delay {r['dead_time_s'] if r['dead_time_s'] is None else round(r['dead_time_s'], 3)} s")
        if abs(r["moved_raw"]) > 2 * noise:
            if r["moved_raw"] > 0 and up_value is None:
                up_value = v
            if r["moved_raw"] < 0 and down_value is None:
                down_value = v
        ctx.sleep(0.3)
        if abs(link.enc[1] - start_raw) > 4 * max(abs(r["moved_raw"]), 10):
            go_back(start_raw)
    go_back(start_raw)

    moving = [r for r in results if abs(r["moved_raw"]) > 2 * noise]
    if not moving:
        raise RuntimeError("no D value moved the encoder: steering powered? encoder regex right?")
    low_dir = np.sign(np.mean([r["moved_raw"] for r in moving if r["value"] < stop] or [0]))
    high_dir = np.sign(np.mean([r["moved_raw"] for r in moving if r["value"] > stop] or [0]))
    if low_dir == 0 or high_dir == 0 or low_dir == high_dir:
        raise RuntimeError(f"both sides of {stop} moved the same way (or one side never moved): "
                           f"low {low_dir}, high {high_dir}. Check the steering wiring.")
    dead = [r["value"] for r in results if abs(r["moved_raw"]) <= 2 * noise]

    def side_info(side):
        rs = [r for r in moving if (r["value"] < stop) == (side == "low") and r["rate_raw_s"]]
        rates = [abs(r["rate_raw_s"]) for r in rs]
        if not rates:
            return None, "unknown", rs
        mode = "constant" if max(rates) / max(min(rates), 1e-6) < 1.25 else "proportional"
        best = max(rs, key=lambda r: (round(abs(r["rate_raw_s"]) / max(rates), 1),
                                      abs(r["value"] - stop)))
        return best["value"], mode, rs
    low_best, low_mode, _ = side_info("low")
    high_best, high_mode, _ = side_info("high")
    mode = low_mode if low_mode == high_mode else f"low side {low_mode}, high side {high_mode}"

    # ---- confirm LEFT by eye
    ctx.set_phase("confirm left by eye")
    ctx.log(f"Watch the front wheels. Next pulse uses D={low_best}.", "step")

    def watch():
        r0 = link.enc[1]
        steer.raw_command(low_best, note="confirm pulse")
        ctx.sleep(min(a.confirm_s, ctx.cfg["safety"]["uncalibrated_pulse_s"]))
        steer.stop(note="confirm stop")
        return link.enc[1] - r0
    run_trial(ctx, f"watch pulse D={low_best}", watch)
    low_is_left = ctx.ch.confirm(f"Did the front wheels turn to the driver's LEFT with D={low_best}?")
    go_back(start_raw)

    left_value, right_value = (low_best, high_best) if low_is_left else (high_best, low_best)
    left_dir = low_dir if low_is_left else high_dir
    out = {
        "left_value": int(left_value), "right_value": int(right_value), "stop_value": stop,
        "left_ticks_sign": int(left_dir), "mode": mode, "no_motion_values": dead,
        "confirmed_by_operator": True,
        "pulses": [{k: r[k] for k in ("value", "moved_raw", "rate_raw_s", "dead_time_s")}
                   for r in results],
    }
    out = clean(out)
    cal.save("steer_map", out, rec.dir)
    ctx.log(f"LEFT = D{left_value}, RIGHT = D{right_value}, steering is {mode}; turning left "
            f"{'increases' if left_dir > 0 else 'decreases'} the encoder count", "ok")
    return out


if __name__ == "__main__":
    run("02_steer_command_map", "Which D values steer left/right; constant or proportional",
        body, add_args)
