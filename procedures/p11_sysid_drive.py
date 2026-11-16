#!/usr/bin/env python3
"""p11 system-identification drive: a random left/right/stop sequence for model fitting.

The data for fitting (and later validating) the steering actuator model and for training the
RL controller in the digital twin. Commands are random with random hold times; a share of
them (--p-short) follow the previous command in under 0.2 s on purpose, to sample dropped
commands in realistic use. Positions stay inside --amplitude of the soft range.

Standstill by default (actuator only). --throttle N drives too (needs VectorNav and a big
open area: the kart wanders with radii down to ~6 m). Runs in --segment second blocks so
releasing SPACE only redoes one block. Writes commands.csv (every command with its intended
hold) next to the usual serial/VectorNav logs.
"""
import random
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.analysis import clean  # noqa: E402
from kart.harness import (hold_brake_for_steering, run, run_trial, set_brake,  # noqa: E402
                          set_throttle, stop_vehicle)


def add_args(ap):
    ap.add_argument("--duration", type=float, default=60.0, help="total seconds of excitation")
    ap.add_argument("--segment", type=float, default=10.0)
    ap.add_argument("--throttle", type=int, default=0)
    ap.add_argument("--min-hold", type=float, default=0.2)
    ap.add_argument("--max-hold", type=float, default=1.2)
    ap.add_argument("--p-short", type=float, default=0.2,
                    help="share of commands sent 0.05-0.2 s after the previous one")
    ap.add_argument("--amplitude", type=float, default=0.9, help="fraction of the soft range")
    ap.add_argument("--seed", type=int, default=None)


def body(ctx):
    a, link, steer, cal, rec, guard = ctx.args, ctx.link, ctx.steer, ctx.cal, ctx.rec, ctx.guard
    rnd = random.Random(a.seed)
    lo, hi = cal.soft_limits()
    lo, hi = a.amplitude * lo, a.amplitude * hi
    driving = a.throttle > 0
    if driving:
        ctx.release_brake_at_end = True
        if not ctx.ch.confirm(f"Driving at throttle {a.throttle} with random steering for "
                              f"{a.duration:.0f} s. Large open area clear? Ready?"):
            return {"skipped": True}
    else:
        hold_brake_for_steering(ctx)
    rec.table("commands", ["t", "command", "hold_s", "short", "pos", "speed_kmh"])
    counts = {"left": 0, "right": 0, "stop": 0, "short": 0, "blocked": 0}
    n_seg = max(1, int(round(a.duration / a.segment)))

    def segment(k):
        if driving:
            if link.cmd.brake:
                set_brake(ctx, False)
            set_throttle(ctx, a.throttle)
        end = time.monotonic() + a.segment
        current = "stop"
        while time.monotonic() < end:
            p = steer.pos()
            choices = ["left", "right", "stop"]
            if p >= hi:
                choices.remove("left")
            if p <= lo:
                choices.remove("right")
            choices = [c for c in choices if c != current] or ["stop"]
            cmd = rnd.choice(choices)
            short = rnd.random() < a.p_short
            hold = rnd.uniform(0.05, 0.2) if short else rnd.uniform(a.min_hold, a.max_hold)
            t = steer.command(cmd, force=True, note=f"sysid {cmd}")
            if t is None:
                counts["blocked"] += 1
                cmd = "stop"
                t = steer.command("stop", force=True, note="sysid stop (limit)")
            counts[cmd] += 1
            counts["short"] += short
            current = cmd
            rec.row("commands", [rec.rel(t), cmd, round(hold, 3), int(short), p, ctx.vn.speed_kmh()])
            t_end = t + hold
            while time.monotonic() < t_end:
                ctx.sleep(0.005)
                if current != "stop" and guard.limit_hit == current:
                    break
        steer.stop(note="segment end", force=False)
        if driving:
            set_throttle(ctx, 0)

    for k in range(1, n_seg + 1):
        ctx.set_phase(f"segment {k}/{n_seg}")
        run_trial(ctx, f"segment {k}", lambda: segment(k))
    if driving:
        stop_vehicle(ctx, note="sysid end")
    run_trial(ctx, "back to centre", lambda: steer.move_to(0))
    pos = np.array([cal.pos(r) for _, r in link.samples])
    hist, edges = np.histogram(pos, bins=10, range=cal.soft_limits())
    out = clean({"segments": n_seg, "counts": counts, "throttle": a.throttle,
                 "position_coverage": {"edges": edges.tolist(), "counts": hist.tolist()}})
    ctx.log(f"sent {counts['left']} left, {counts['right']} right, {counts['stop']} stop "
            f"({counts['short']} within 0.2 s of the previous)", "ok")
    return out


if __name__ == "__main__":
    run("11_sysid_drive", "Random steering sequence for model fitting", body, add_args,
        needs=("steer_map", "zero", "range"), needs_vn=lambda a: a.throttle > 0)
