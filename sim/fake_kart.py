#!/usr/bin/env python3
"""Fake kart on virtual serial ports, for dry runs of the tests and the dashboard.

    python3 sim/fake_kart.py            # drive-by-wire port only
    python3 sim/fake_kart.py --vn       # plus a fake VectorNav (ASCII $VNINS / $VNYMR)

It prints the port paths and links them as sim/run/dbw and sim/run/vn. Point the tests at it:

    python3 dashboard/app.py --config sim/sim_config.toml

The model has what kartContext.md describes: steering only turns left / right / stops, a
command takes effect ~0.2 s after it arrives, a command within 0.2 s of the previous one may
be dropped, left and right rates differ, the wheel locks at +-15 deg, and the encoder is an
incremental count on the steering wheel (random offset at start). Plus backlash between the
steering wheel and the road wheels, throttle -> speed with lag, brake, and a bicycle model
for position. All numbers are invented; change them with the options below.

The true state is written to sim/run/state.json (road wheel angle etc.) so you can answer the
"enter the wheel angle" prompts during a dry run.
"""
import argparse
import json
import math
import os
import pty
import random
import re
import select
import time
import tty
from pathlib import Path

FRAME_RE = re.compile(rb"\*A(\d+)B(\d+)C(\d+)D(\d+)E(\d+)F(\d+)G(\d+)H(\d+)I(\d+)J(\d+)#")
HERE = Path(__file__).resolve().parent
RUN = HERE / "run"


def make_pty(name):
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    RUN.mkdir(parents=True, exist_ok=True)
    link = RUN / name
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(path)
    return master, slave, path, link


def nmea(body):
    cs = 0
    for ch in body.encode():
        cs ^= ch
    return f"${body}*{cs:02X}\r\n".encode()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--vn", action="store_true", help="also make a fake VectorNav port")
    ap.add_argument("--latency", type=float, default=0.20, help="command delay (s)")
    ap.add_argument("--jitter", type=float, default=0.03, help="std dev of the delay (s)")
    ap.add_argument("--drop-window", type=float, default=0.20,
                    help="a command this soon after the last accepted one may be dropped")
    ap.add_argument("--drop-prob", type=float, default=0.5)
    ap.add_argument("--rate-left", type=float, default=12.0, help="road wheel deg/s turning left")
    ap.add_argument("--rate-right", type=float, default=9.0, help="road wheel deg/s turning right")
    ap.add_argument("--max-wheel", type=float, default=15.0, help="road wheel lock (deg)")
    ap.add_argument("--ratio", type=float, default=18.0, help="steering wheel deg per road wheel deg")
    ap.add_argument("--cpr", type=int, default=2400, help="encoder counts per steering wheel turn")
    ap.add_argument("--enc-hz", type=float, default=50.0, help="encoder lines per second")
    ap.add_argument("--enc-format", default="ENC:{ticks}", help="encoder line format")
    ap.add_argument("--respond-only", action="store_true",
                    help="only print the encoder after receiving a frame")
    ap.add_argument("--left-below", type=int, default=1,
                    help="1: D < 50 turns left, 0: D > 50 turns left")
    ap.add_argument("--deadband", type=int, default=3, help="|D-50| at or below this does nothing")
    ap.add_argument("--proportional", action="store_true", help="rate scales with |D-50|")
    ap.add_argument("--backlash", type=float, default=0.6, help="road wheel backlash (deg)")
    ap.add_argument("--wheelbase", type=float, default=1.6)
    ap.add_argument("--kmh-per-throttle", type=float, default=0.6,
                    help="steady km/h per throttle unit above 50")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--run-dir", default=None, help="where to put the port links and state.json")
    a = ap.parse_args()
    global RUN
    if a.run_dir:
        RUN = Path(a.run_dir).resolve()
    rnd = random.Random(a.seed)

    dbw_m, dbw_s, dbw_path, dbw_link = make_pty("dbw")
    print(f"drive-by-wire: {dbw_path}  (link {dbw_link})", flush=True)
    vn_m = None
    if a.vn:
        vn_m, vn_s, vn_path, vn_link = make_pty("vn")
        print(f"vectornav:     {vn_path}  (link {vn_link})", flush=True)

    ticks_per_sw_deg = a.cpr / 360.0
    enc_offset = rnd.randint(-5000, 5000)
    max_sw = a.max_wheel * a.ratio
    sw = 0.0              # steering wheel angle (deg, left +)
    road = 0.0            # road wheel angle with backlash
    motor = 0.0           # steering wheel rate (deg/s)
    motor_target = 0.0
    cmd = {"D": 50, "B": 0, "I": 0}
    pending = []          # (apply_time, frame dict)
    last_accept = -1.0
    speed = 0.0
    x = y = 0.0
    psi = math.radians(90.0)    # heading, ENU (CCW from east); start facing north
    lat0, lon0 = 15.39, 73.88
    buf = b""
    t_prev = time.monotonic()
    next_enc = next_vn = next_state = t_prev
    stats = {"frames": 0, "dropped": 0}

    def apply(f):
        nonlocal motor_target
        cmd.update(f)
        d = f["D"] - 50
        if abs(d) <= a.deadband:
            motor_target = 0.0
        else:
            left = (d < 0) == bool(a.left_below)
            rate = (a.rate_left if left else a.rate_right) * a.ratio
            if a.proportional:
                rate *= abs(d) / 50.0
            motor_target = rate if left else -rate

    def write(fd, data):
        try:
            os.write(fd, data)
        except (BlockingIOError, OSError):
            pass

    while True:
        fds = [dbw_m]
        r, _, _ = select.select(fds, [], [], 0.002)
        now = time.monotonic()
        if dbw_m in r:
            try:
                buf += os.read(dbw_m, 4096)
            except (BlockingIOError, OSError):
                pass
            for m in FRAME_RE.finditer(buf):
                g = [int(v) for v in m.groups()]
                f = {"D": g[3], "B": g[1], "I": g[8]}
                stats["frames"] += 1
                if now - last_accept < a.drop_window and rnd.random() < a.drop_prob:
                    stats["dropped"] += 1
                else:
                    last_accept = now
                    pending.append((now + max(0.02, rnd.gauss(a.latency, a.jitter)), f))
                if a.respond_only:
                    write(dbw_m, (a.enc_format.format(ticks=round(sw * ticks_per_sw_deg)
                                                      + enc_offset) + "\r\n").encode())
            idx = buf.rfind(b"#")
            if idx >= 0:
                buf = buf[idx + 1:]
            if len(buf) > 1000:
                buf = b""
        pending.sort(key=lambda p: p[0])
        while pending and pending[0][0] <= now:
            apply(pending.pop(0)[1])

        dt = now - t_prev
        t_prev = now
        # steering motor: quick first-order response, stalls at the locks
        motor += (motor_target - motor) * min(1.0, dt / 0.05)
        sw += motor * dt
        if abs(sw) > max_sw:
            sw = math.copysign(max_sw, sw)
            motor = 0.0
        target_road = sw / a.ratio
        if target_road - road > a.backlash / 2:
            road = target_road - a.backlash / 2
        elif target_road - road < -a.backlash / 2:
            road = target_road + a.backlash / 2

        # longitudinal
        v_target = max(0.0, (cmd["B"] - 50) * a.kmh_per_throttle / 3.6)
        if cmd["I"]:
            speed = max(0.0, speed - 3.0 * dt)
        elif speed < v_target:
            speed = min(v_target, speed + 1.2 * dt)
        else:
            speed = max(v_target, speed - 0.4 * dt)
        yaw_rate = speed * math.tan(math.radians(road)) / a.wheelbase
        psi += yaw_rate * dt
        x += speed * math.cos(psi) * dt
        y += speed * math.sin(psi) * dt

        if not a.respond_only and now >= next_enc:
            next_enc = now + 1.0 / a.enc_hz
            ticks = round(sw * ticks_per_sw_deg) + enc_offset
            write(dbw_m, (a.enc_format.format(ticks=ticks) + "\r\n").encode())
        if vn_m is not None and now >= next_vn:
            next_vn = now + 0.05
            lat = lat0 + math.degrees(y / 6378137.0)
            lon = lon0 + math.degrees(x / (6378137.0 * math.cos(math.radians(lat0))))
            yaw_vn = (90.0 - math.degrees(psi)) % 360.0
            vn_, ve_ = speed * math.sin(psi), speed * math.cos(psi)
            write(vn_m, nmea(f"VNINS,{now:.3f},2400,0006,{yaw_vn:+.3f},+0.000,+0.000,"
                             f"{lat:+.8f},{lon:+.8f},+010.000,{vn_:+.3f},{ve_:+.3f},+0.000,"
                             f"0.5,0.3,0.05"))
            write(vn_m, nmea(f"VNYMR,{yaw_vn:+.3f},+0.000,+0.000,+0.10,+0.00,+0.40,"
                             f"+0.00,+0.00,-9.81,+0.00,+0.00,{-yaw_rate:+.5f}"))
        if now >= next_state:
            next_state = now + 0.2
            (RUN / "state.json").write_text(json.dumps({
                "road_wheel_deg": round(road, 2), "steering_wheel_deg": round(sw, 1),
                "encoder": round(sw * ticks_per_sw_deg) + enc_offset,
                "speed_kmh": round(speed * 3.6, 2), "x": round(x, 2), "y": round(y, 2),
                "D": cmd["D"], "B": cmd["B"], "I": cmd["I"], **stats}))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
