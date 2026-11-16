"""Steering helpers on top of the left / right / stop commands.

The kart's steering motor only turns left, turns right, or stops (holding the angle), and it
acts on a command ~0.2 s after it arrives; a second command inside that window may be lost.
So: STOP is re-sent until the encoder shows the wheel has stopped, normal commands keep at
least min_command_gap_s apart, and moves stop early by the expected overshoot ("lead").
"""
import time

from .channel import Aborted


class Steering:
    def __init__(self, ctx):
        self.ctx, self.link, self.cal, self.cfg = ctx, ctx.link, ctx.cal, ctx.cfg
        st = self.cfg["steering"]
        self.gap = st["min_command_gap_s"]
        self.assumed_stop_latency = st["assumed_stop_latency_s"]
        self.stop_retry = st["stop_retry_s"]
        self.noise = self.cfg["encoder"]["noise_ticks"]
        self.observed_rate = {}      # fastest rate seen per direction this run (ticks/s)

    # -------------------------------------------------------------- position
    def pos(self):
        return self.cal.pos(self.link.enc[1])

    def deg(self):
        return self.cal.deg(self.pos())

    def value(self, direction):
        return {"left": self.cal.left_value, "right": self.cal.right_value,
                "stop": self.cal.stop_value}[direction]

    def rate_now(self, window=0.15):
        """Current rate in pos ticks/s (left positive), from the last `window` seconds."""
        s = self.link.samples_between(time.monotonic() - window)
        if len(s) < 2 or s[-1][0] - s[0][0] < 1e-3:
            return 0.0
        return (self.cal.pos(s[-1][1]) - self.cal.pos(s[0][1])) / (s[-1][0] - s[0][0])

    def typical_rate(self, direction):
        sl = self.cal.get("slew")
        if sl and sl.get(direction, {}).get("rate_ticks_s"):
            return abs(sl[direction]["rate_ticks_s"])
        return self.observed_rate.get(direction)

    def stop_lead_ticks(self, direction):
        """How far the wheel keeps going after a STOP is sent."""
        lat = self.cal.get("latency")
        if lat and lat.get(direction, {}).get("overshoot_ticks") is not None:
            return abs(lat[direction]["overshoot_ticks"])
        rate = abs(self.rate_now()) or (self.typical_rate(direction) or 0)
        return rate * self.assumed_stop_latency

    def min_step_ticks(self, direction):
        rate = self.typical_rate(direction)
        return None if rate is None else rate * self.gap

    def default_tol(self):
        tpd = self.cal.ticks_per_deg()
        st = self.cfg["steering"]
        return st["move_tolerance_deg"] * tpd if tpd else st["move_tolerance_ticks"]

    # -------------------------------------------------------------- commands
    def wait_gap(self):
        t = self.link.last_cmd_tx_t
        if t is not None:
            remaining = self.gap - (time.monotonic() - t)
            if remaining > 0:
                self.ctx.sleep(remaining)

    def command(self, direction, force=False, note=""):
        """Send left/right/stop. Returns the send time, or None if a soft limit blocks it."""
        self.ctx.guard.check()
        if not force:
            self.wait_gap()
        lim = self.cal.soft_limits()
        p = self.pos()
        if lim and p is not None and self.cal.zero_raw is not None:
            if (direction == "left" and p >= lim[1]) or (direction == "right" and p <= lim[0]):
                self.ctx.log(f"not steering {direction}: already at the soft limit (pos {p})", "warn")
                return None
        t = self.link.send(steering=self.value(direction), note=note or f"steer {direction}")
        self.ctx.guard.note_steer(direction, t)
        return t

    def raw_command(self, value, note="", force=False):
        """Send a raw D value (before the left/right mapping is known)."""
        self.ctx.guard.check()
        if not force:
            self.wait_gap()
        t = self.link.send(steering=int(value), note=note or f"steer raw {value}")
        self.ctx.guard.note_steer("stop" if value == self.cal.stop_value else f"raw{value}", t)
        return t

    def stop(self, note="stop", confirm=True, force=True):
        t = self.command("stop", force=force, note=note)
        if confirm:
            self.confirm_stopped(t)
        return t

    def is_quiet(self, quiet_s):
        s = self.link.samples_between(time.monotonic() - quiet_s)
        if not s:
            return True
        vals = [r for _, r in s]
        return max(vals) - min(vals) <= self.noise

    def confirm_stopped(self, t_stop, quiet_s=0.3, timeout=3.0):
        """Wait until the encoder is still; re-send STOP if it keeps moving."""
        last = t_stop
        while True:
            self.ctx.sleep(0.02)
            now = time.monotonic()
            if now - t_stop >= quiet_s and self.is_quiet(quiet_s):
                return self.pos()
            if now - last > self.stop_retry:
                self.ctx.rec.event("stop_retry", since_stop=round(now - t_stop, 3))
                self.link.send(steering=self.cal.stop_value, note="stop retry")
                last = now
            if now - t_stop > timeout:
                raise Aborted("steering did not stop after repeated STOP commands")

    def wait_quiet(self, quiet_s=0.3, timeout=3.0):
        t0 = time.monotonic()
        while not self.is_quiet(quiet_s) or time.monotonic() - t0 < quiet_s:
            self.ctx.sleep(0.02)
            if time.monotonic() - t0 > timeout:
                break
        return self.pos()

    def move_to(self, target, tol=None, max_iter=6, note=""):
        """Bang-bang move to `target` pos ticks. Returns the final pos."""
        tol = self.default_tol() if tol is None else tol
        lim = self.cal.soft_limits()
        if lim:
            target = min(max(target, lim[0]), lim[1])
        for _ in range(max_iter):
            p = self.pos()
            err = target - p
            if abs(err) <= tol:
                break
            d = "left" if err > 0 else "right"
            step = self.min_step_ticks(d)
            if step and abs(err) < 0.6 * step:
                break                         # closer than one minimum step can reach
            t0 = self.command(d, note=note or f"move_to {target:.0f}")
            if t0 is None:
                break
            sgn = 1 if d == "left" else -1
            while True:
                self.ctx.sleep(0.005)
                r = abs(self.rate_now())
                if r > self.observed_rate.get(d, 0):
                    self.observed_rate[d] = r
                p = self.pos()
                remaining = (target - p) * sgn
                if self.ctx.guard.limit_hit == d:
                    break
                if remaining <= self.stop_lead_ticks(d) and time.monotonic() - t0 >= self.gap:
                    break
                if time.monotonic() - t0 > self.ctx.guard.max_move_s():
                    break
            self.stop(note="move_to stop", force=False)
        return self.pos()
