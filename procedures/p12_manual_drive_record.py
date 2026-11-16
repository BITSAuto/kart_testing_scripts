#!/usr/bin/env python3
"""p12 manual drive from the dashboard keyboard, everything recorded.

    A / D (hold)   steer left / right; release stops the steering
    W              throttle on at --throttle      S   throttle off
    B              brake on / off                 ENTER  end the drive
    ESC / X        abort (stop + brake)

A terminal can only auto-repeat one held key, so steering and throttle can't both be "held".
Throttle is a toggle instead, with a keep-alive: if no key at all is pressed for --keepalive
seconds while the throttle is on, it cuts the throttle and brakes. Tap any key (or keep
steering) to keep going. Dashboard only.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.harness import run, set_throttle  # noqa: E402


def add_args(ap):
    ap.add_argument("--throttle", type=int, default=0, help="throttle value for W (0 = W disabled)")
    ap.add_argument("--keepalive", type=float, default=3.0)
    ap.add_argument("--duration", type=float, default=900.0, help="longest session (s)")


def body(ctx):
    a, link, steer, ch, guard, rec = ctx.args, ctx.link, ctx.steer, ctx.ch, ctx.guard, ctx.rec
    if not ch.tui:
        raise RuntimeError("manual drive needs the dashboard")
    sc = ctx.cfg["safety"]
    ctx.release_brake_at_end = True
    ch.log("A/D hold = steer, W = throttle on, S = off, B = brake, ENTER = end", "step")
    ch.status(jog=True, text="manual drive")
    t_end = time.monotonic() + a.duration
    current = "stop"
    with guard.jogging():
        while time.monotonic() < t_end:
            guard.check()
            want = ch.jog_state(sc["jog_release_s"], sc["jog_initial_s"])
            if want != current:
                if current != "stop" or want == "stop":
                    steer.command("stop", force=True, note="manual stop")
                if want != "stop":
                    steer.command(want, note=f"manual {want}")
                current = want
            while not ch.keys.empty():
                _, k = ch.keys.get_nowait()
                rec.event("key", key=k)
                if k == "enter":
                    t_end = 0
                elif k == "w" and a.throttle > 0:
                    if link.cmd.brake:
                        link.send(brake=0, note="manual brake off")
                    set_throttle(ctx, a.throttle, "manual throttle on")
                elif k == "s":
                    set_throttle(ctx, 0, "manual throttle off")
                elif k == "b":
                    link.send_reliable(brake=0 if link.cmd.brake else 1, throttle=0,
                                       note="manual brake toggle")
            if link.cmd.throttle > 0 and time.monotonic() - ch.last_input_t > a.keepalive:
                link.send_reliable(throttle=0, brake=1, note="keep-alive expired")
                ch.log(f"no key for {a.keepalive} s: throttle cut, brake on", "warn")
            time.sleep(0.02)
        link.send_reliable(steering=ctx.cal.stop_value, throttle=0, note="manual end")
    ch.status(jog=False)
    return {"duration_s": round(time.monotonic() - rec.t0, 1)}


if __name__ == "__main__":
    run("12_manual_drive_record", "Keyboard driving, everything recorded", body, add_args,
        needs=("steer_map",))
