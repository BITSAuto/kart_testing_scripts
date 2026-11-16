"""Common runner for every procedure: arguments, link, recorder, safety, telemetry, cleanup.

A procedure is a function body(ctx) -> summary dict, started with

    run("04_slew_rate", "description", body, add_args=..., needs=("steer_map", "zero", "range"))

Whatever happens (finish, abort, crash), the kart gets STOP + throttle 0 at the end, and the
run directory gets meta.json with the status and summary.json with whatever was measured.
"""
import argparse
import json
import platform
import signal
import sys
import threading
import time
import traceback

from .channel import Aborted, Channel, Paused  # noqa: F401
from .config import Calibration, load_config
from .guard import Guard
from .link import DbwLink
from .recorder import Recorder
from .steering import Steering
from .vectornav import make_vectornav


class Ctx:
    def __init__(self, **kw):
        self.__dict__.update(kw)
        self.summary = {}
        self.phase = ""
        self.steer = None
        self.guard = None

    def sleep(self, dt):
        """Sleep while checking for abort / deadman faults."""
        end = time.monotonic() + dt
        while True:
            self.guard.check()
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(left, 0.01))

    def log(self, msg, level="info"):
        self.ch.log(msg, level)

    def set_phase(self, phase, **extra):
        self.phase = phase
        self.ch.status(phase=phase, **extra)
        self.rec.event("phase", phase=phase, **extra)


def run_trial(ctx, label, fn, retries=3):
    """Run fn() inside a motion block; redo it if the deadman was released."""
    for attempt in range(retries + 1):
        try:
            with ctx.guard.motion(label):
                ctx.rec.event("trial_start", label=label, attempt=attempt)
                out = fn()
                ctx.rec.event("trial_end", label=label)
                return out
        except Paused as e:
            ctx.rec.event("trial_interrupted", label=label, reason=str(e))
            ctx.log(f"'{label}' interrupted ({e}); it will be redone", "warn")
            ctx.guard.clear_fault()
    raise Aborted(f"'{label}' interrupted {retries + 1} times")


def jog_session(ctx, instruction):
    """Let the operator steer by hand: hold A/D in the dashboard, ENTER when done."""
    ch, steer, sc = ctx.ch, ctx.steer, ctx.cfg["safety"]
    ctx.rec.event("jog_start", instruction=instruction)
    with ctx.guard.jogging():
        if not ch.tui:
            ctx.log(instruction, "step")
            while True:
                s = ch.ask("jog: 'l 0.3' / 'r 0.3' = steer left/right for 0.3 s, empty = done",
                           default="")
                if not s:
                    break
                parts = s.split()
                d = {"l": "left", "r": "right"}.get(parts[0][:1].lower())
                dur = float(parts[1]) if len(parts) > 1 else 0.3
                if d:
                    steer.command(d, note="jog")
                    ctx.sleep(min(dur, sc["max_move_s"]))
                    steer.stop(note="jog stop", force=False)
                ctx.log(f"pos {steer.pos()}  raw {ctx.link.enc[1]}")
        else:
            ch.status(jog=True, text=instruction)
            ctx.log(instruction + "  [hold A = left, D = right, release = stop, ENTER = done]",
                    "step")
            while not ch.keys.empty():
                ch.keys.get_nowait()
            current = "stop"
            t_start = time.monotonic()
            while True:
                ctx.guard.check()
                want = ch.jog_state(sc["jog_release_s"], sc["jog_initial_s"])
                if want != current:
                    if current != "stop" or want == "stop":
                        steer.command("stop", force=True, note="jog stop")
                    if want != "stop":
                        steer.command(want, note=f"jog {want}")
                    current = want
                done = False
                while not ch.keys.empty():
                    tk, k = ch.keys.get_nowait()
                    if k == "enter" and tk - t_start > 0.3:
                        done = True
                if done and current == "stop":
                    break
                time.sleep(0.02)
            ch.status(jog=False)
        steer.stop(note="jog end")
    ctx.rec.event("jog_end", raw=ctx.link.enc[1])


def set_throttle(ctx, value, note=""):
    """Throttle like ver1.py: B=prearm first (unless going to 0 or already there), then value."""
    link, pre = ctx.link, ctx.cfg["throttle"]["prearm"]
    value = int(max(0, min(value, ctx.cfg["throttle"]["max"])))
    if value == 0:
        return link.send_reliable(throttle=0, note=note or "throttle 0")
    ctx.guard.check()
    if pre and link.cmd.throttle != pre and value != pre:
        ctx.steer.wait_gap()
        link.send(throttle=pre, note="throttle prearm")
    ctx.steer.wait_gap()
    return link.send(throttle=value, note=note or f"throttle {value}")


def set_brake(ctx, on, note=""):
    ctx.steer.wait_gap()
    return ctx.link.send_reliable(brake=1 if on else 0,
                                  note=note or f"brake {'on' if on else 'off'}")


def stop_vehicle(ctx, brake=True, timeout=25.0, note="stop vehicle"):
    """Throttle 0 (+ brake), reliably, then wait until VectorNav shows standstill. Re-sends
    every second while still moving (a dropped frame must not leave the kart driving)."""
    link, vn = ctx.link, ctx.vn
    link.send_reliable(throttle=0, brake=1 if brake else link.cmd.brake, note=note)
    if not vn.available():
        ctx.sleep(1.0)
        return
    t0 = last = time.monotonic()
    while (vn.speed_kmh() or 0) > 0.3:
        ctx.sleep(0.05)
        now = time.monotonic()
        if now - last > 1.0:
            link.send(throttle=0, brake=1 if brake else link.cmd.brake, note=f"{note} (resend)")
            last = now
        if now - t0 > timeout:
            raise Aborted(f"still moving {timeout:.0f} s after {note}")


def hold_brake_for_steering(ctx):
    """Steering tests at standstill: brake on, released again at the end if all went well."""
    if ctx.cfg["steering"].get("hold_brake_in_steering_tests", True):
        set_brake(ctx, True, "brake on for steering test")
        ctx.release_brake_at_end = True


def _telemetry(ctx, stop_evt):
    link, cal, vn, ch, guard = ctx.link, ctx.cal, ctx.vn, ctx.ch, ctx.guard
    last = (time.monotonic(), link.rx_count, link.enc_count, vn.count)
    hz = (0.0, 0.0, 0.0)
    while not stop_evt.wait(0.05):
        now = time.monotonic()
        if now - last[0] >= 1.0:
            dt = now - last[0]
            hz = ((link.rx_count - last[1]) / dt, (link.enc_count - last[2]) / dt,
                  (vn.count - last[3]) / dt)
            last = (now, link.rx_count, link.enc_count, vn.count)
        raw = link.enc[1]
        pos = cal.pos(raw) if (raw is not None and cal.zero_raw is not None) else None
        st = vn.state if vn.available() else {}
        c = link.cmd
        ch.emit("tel", t=round(ctx.rec.rel(now), 3), raw=raw, pos=pos,
                deg=None if pos is None else cal.deg(pos), sd=c.steering, dir=guard.move_dir,
                thr=c.throttle, brk=c.brake, v=None if st.get("speed_mps") is None else
                round(st["speed_mps"] * 3.6, 2), yr=st.get("yaw_rate"), x=st.get("x_m"),
                y=st.get("y_m"), fix=st.get("fix"), rx_hz=round(hz[0], 1), enc_hz=round(hz[1], 1),
                vn_hz=round(hz[2], 1), enc_age=None if link.enc_age() is None else
                round(link.enc_age(), 2), dm=ch.deadman_held(), need_dm=guard.motion_active and
                guard.deadman_enabled, jog=guard.jog_active, fault=guard.fault, phase=ctx.phase,
                line=link.last_lines[-1][2][:80] if link.last_lines else "")


def run(test, description, body, add_args=None, motion=True, needs=(), needs_vn=False,
        send_neutral=True, open_link=True):
    """motion / needs_vn may be functions of the parsed args."""
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument("--config", help="config file (default: config.toml)")
    ap.add_argument("--port", help="override serial.port")
    ap.add_argument("--label", default="", help="added to the run directory name")
    ap.add_argument("--notes", default="", help="free text stored in meta.json")
    ap.add_argument("--tui", action="store_true", help="talk to the dashboard (set by it)")
    ap.add_argument("--no-deadman", action="store_true",
                    help="run motion without the dashboard deadman (you must hold the brake / e-stop)")
    ap.add_argument("--force", action="store_true", help="run even if calibrations are missing")
    if add_args:
        add_args(ap)
    args = ap.parse_args()

    cfg = load_config(args.config)
    if args.port:
        cfg["serial"]["port"] = args.port
    ch = Channel(args.tui, cfg)
    ch.emit("start", test=test, description=description)

    if callable(motion):
        motion = motion(args)
    if callable(needs_vn):
        needs_vn = needs_vn(args)
    if callable(needs):
        needs = needs(args)
    if motion and not args.tui and not args.no_deadman:
        ch.log("This procedure moves the kart. Run it from the dashboard (deadman on SPACE), or "
               "add --no-deadman if someone has a hand on the brake / e-stop.", "error")
        sys.exit(2)
    cal = Calibration(cfg)
    missing = [n for n in needs if not cal.has(n)]
    if missing and not args.force:
        ch.log(f"Missing calibration: {', '.join(missing)}. Run the procedures that make them "
               "first (see README), or add --force.", "error")
        ch.emit("result", status="error", error=f"missing calibration {missing}")
        sys.exit(2)

    data_root = cfg["_data_root"]
    rec = Recorder(data_root, test, args.label, meta={
        "description": description, "argv": sys.argv, "args": vars(args),
        "notes": args.notes, "host": platform.node(), "python": platform.python_version(),
        "config": {k: v for k, v in cfg.items() if not k.startswith("_")},
        "config_path": cfg["_path"], "calibration": cal.data,
    })
    ch.emit("run_dir", path=str(rec.dir))
    ch.log(f"{test}: recording to {rec.dir}", "step")

    link = DbwLink(cfg, rec, ch.log)
    vn = make_vectornav(cfg, rec)
    ctx = Ctx(cfg=cfg, args=args, ch=ch, rec=rec, link=link, cal=cal, vn=vn)
    status, error = "completed", None
    stop_evt = threading.Event()

    def on_term(signum, frame):
        ch.abort(f"signal {signum}")
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)
    try:
        if open_link:
            try:
                link.open(send_neutral=False)
            except Exception as e:
                raise RuntimeError(f"cannot open {cfg['serial']['port']}: {e}") from e
        try:
            vn.start()
        except Exception as e:
            ch.log(f"VectorNav ({cfg['vectornav']['source']}) not started: {e}", "warn")
        ctx.steer = Steering(ctx)
        ctx.guard = Guard(ctx)
        threading.Thread(target=_telemetry, args=(ctx, stop_evt), daemon=True).start()
        if send_neutral and open_link:
            link.send(note="neutral at start")
        if needs_vn:
            t0 = time.monotonic()
            while not vn.available() and time.monotonic() - t0 < 3.0:
                time.sleep(0.05)
            if not vn.available():
                raise RuntimeError("this procedure needs VectorNav data and none is arriving "
                                   f"(vectornav.source = {cfg['vectornav']['source']})")
        result = body(ctx)
        if result is not None:
            ctx.summary = result
    except (Aborted, KeyboardInterrupt) as e:
        status = "aborted"
        error = str(e) or ch.abort_reason or "interrupted"
        ch.log(f"ABORTED: {error}", "error")
    except Exception as e:
        status, error = "error", f"{type(e).__name__}: {e}"
        ch.log(f"ERROR: {error}", "error")
        ch.log(traceback.format_exc())
    finally:
        try:
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            if ctx.guard:
                ctx.guard.shutdown()
            if link.ser and link.ser.is_open:
                moving = link.cmd.steering != cal.stop_value or link.cmd.throttle > 0
                brake = link.cmd.brake
                if status == "completed" and getattr(ctx, "release_brake_at_end", False):
                    brake = 0
                for i in range(3 if moving or status != "completed" else 1):
                    link.send(steering=cal.stop_value, throttle=0, brake=brake, note="final stop")
                    time.sleep(0.25)
                if brake:
                    ch.log("brake (I=1) left ON", "warn")
        except Exception as e:
            ch.log(f"final stop failed: {e}", "error")
        stop_evt.set()
        for closer in (vn.close, link.close):
            try:
                closer()
            except Exception:
                pass
        summary = dict(ctx.summary or {})
        if error:
            summary["error"] = error
        rec.finish(status, summary)
        ch.emit("result", status=status, summary=summary, run_dir=str(rec.dir))
        ch.log(f"{test}: {status}. Data in {rec.dir}", "ok" if status == "completed" else "warn")
        if summary and not args.tui:
            ch.log(json.dumps(summary, indent=2, default=str))
    sys.exit(0 if status == "completed" else 1)
