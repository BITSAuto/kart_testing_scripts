"""Safety: a 50 Hz watchdog that stops the kart whenever something is off.

Stops (STOP steering, throttle 0, brake if the kart may be rolling, repeated 3x because the
kart can drop commands sent close together) when:
  * the operator aborts, or the dashboard goes away (stdin closes)
  * the deadman (SPACE) is released while a motion block is running   -> Paused
  * steering or throttle is on outside a motion block or jog session    -> Aborted
  * encoder data stops while steering, or the serial link fails         -> Aborted
  * one steering movement lasts too long                               -> Aborted
  * speed above max_speed_kmh, or throttle on too long                  -> Aborted
and stops just the steering (no fault) at the soft limits inside the measured locks.
"""
import threading
import time
from contextlib import contextmanager

from .channel import Aborted, Paused


class Guard:
    def __init__(self, ctx):
        self.ctx = ctx
        self.link, self.ch, self.cal, self.vn = ctx.link, ctx.ch, ctx.cal, ctx.vn
        sc = ctx.cfg["safety"]
        self.sc = sc
        self.stop_value = self.cal.stop_value
        self.deadman_enabled = bool(ctx.ch.tui and sc["deadman"] and not ctx.args.no_deadman)
        self.motion_active = False
        self.jog_active = False
        self.fault = None
        self.fault_kind = None
        self.limit_hit = None
        self.move_dir, self.move_t0 = "stop", None
        self.throttle_t0 = None
        self._no_motion_noted = False
        self._last_rolling_resend = 0.0
        self._repeat_stops = []
        self._running = True
        self._th = threading.Thread(target=self._watch, daemon=True)
        self._th.start()

    # ------------------------------------------------------------- public API
    def check(self):
        """Raise if the procedure must stop. Call often (ctx.sleep does)."""
        if self.ch.abort_evt.is_set():
            if not self.fault:
                self.trip(self.ch.abort_reason or "abort", "abort")
            raise Aborted(self.ch.abort_reason or "abort")
        if self.fault:
            if self.fault_kind == "pause":
                raise Paused(self.fault)
            raise Aborted(self.fault)

    @contextmanager
    def motion(self, label=""):
        """Run a block that moves the kart. Waits for the deadman first."""
        self.check()
        self.wait_for_deadman(label)
        self.motion_active = True
        self.ch.status(motion=True, label=label, deadman=self.deadman_enabled)
        try:
            yield
        finally:
            self.motion_active = False
            self.ch.status(motion=False, label=label, deadman=self.deadman_enabled)
            if self._moving() or self.link.cmd.throttle > 0:
                self.safe_stop(f"end of '{label}'")

    @contextmanager
    def jogging(self):
        self.jog_active = True
        try:
            yield
        finally:
            self.jog_active = False
            if self._moving():
                self.safe_stop("end of jog")

    def wait_for_deadman(self, label=""):
        if not self.deadman_enabled or self.ch.deadman_held():
            return
        self.ch.status(need_deadman=True, label=label)
        self.ch.log(f"Hold SPACE to start: {label}" if label else "Hold SPACE to continue", "step")
        while not (self.ch.deadman_held() and self.ch.hb_streak >= 3):
            if self.ch.abort_evt.is_set():
                raise Aborted(self.ch.abort_reason)
            time.sleep(0.02)
        self.ch.status(need_deadman=False, label=label)

    def clear_fault(self):
        """After a Paused: wait for the repeated stops to go out, then allow motion again."""
        while self._repeat_stops:
            time.sleep(0.05)
        self.fault = self.fault_kind = None

    def note_steer(self, direction, t):
        self._no_motion_noted = False
        if direction != self.move_dir:
            self.move_dir = direction
            self.move_t0 = t if direction != "stop" else None
            self.limit_hit = None

    def max_move_s(self):
        if not self.cal.has("range"):
            return self.sc["uncalibrated_pulse_s"] if not self.jog_active else self.sc["max_move_s"]
        return self.sc["max_move_s"]

    def safe_stop(self, reason, brake=None):
        """Stop steering and throttle (no fault)."""
        if brake is None:
            brake = self.link.cmd.brake
        self.link.send(steering=self.stop_value, throttle=0, brake=brake, note=f"stop: {reason}")
        self.note_steer("stop", time.monotonic())
        self.throttle_t0 = None

    def trip(self, reason, kind="abort"):
        """Emergency stop and fault. kind 'pause' lets the trial be redone."""
        first = self.fault is None
        self.fault, self.fault_kind = reason, kind
        rolling = self.link.cmd.throttle > 0 or (self.vn.speed_kmh() or 0) > 0.5
        brake = 1 if (self.sc["abort_brake"] and rolling) else self.link.cmd.brake
        try:
            self.safe_stop(f"SAFETY {reason}", brake=brake)
        except Exception:
            pass
        now = time.monotonic()
        self._repeat_stops = [now + 0.25, now + 0.5]
        if first:
            self.ctx.rec.event("safety_stop", reason=reason, kind=kind)
            self.ch.log(f"SAFETY STOP: {reason}", "warn" if kind == "pause" else "error")
            self.ch.status(fault=reason, kind=kind)

    def shutdown(self):
        self._running = False
        self._th.join(timeout=0.5)

    # --------------------------------------------------------------- watchdog
    def _moving(self):
        return self.link.cmd.steering != self.stop_value

    def _watch(self):
        while self._running:
            time.sleep(0.02)
            now = time.monotonic()
            if self._repeat_stops and now >= self._repeat_stops[0]:
                self._repeat_stops.pop(0)
                try:
                    self.link.send(steering=self.stop_value, throttle=0, note="stop repeat")
                except Exception:
                    pass
            try:
                self._check_once(now)
            except Exception as e:      # never let the watchdog die
                self.ch.log(f"watchdog error: {e}", "error")

    def _check_once(self, now):
        link = self.link
        moving = self._moving()
        driving = link.cmd.throttle > 0
        if driving and self.throttle_t0 is None:
            self.throttle_t0 = now
        if not driving:
            self.throttle_t0 = None
        if self.ch.abort_evt.is_set():
            if not self.fault or ((moving or driving) and not self._repeat_stops):
                self.trip(self.ch.abort_reason or "abort", "abort")
            return
        if link.fault:
            if not self.fault:
                self.trip(link.fault, "abort")
            return
        if not (moving or driving):
            # Commanded stopped but still rolling after a while: a throttle-0 frame may have
            # been dropped. Re-send the current (stopped) state once a second.
            v = self.vn.speed_kmh()
            if v is not None and v > 1.0 and not (self.motion_active or self.jog_active) and \
                    now - (link.last_cmd_tx_t or 0) > 1.5 and now - self._last_rolling_resend > 1.0:
                self._last_rolling_resend = now
                link.send(throttle=0, note=f"resend: rolling {v:.1f} km/h with throttle 0")
            return
        if self.fault:          # stopped already; repeat if something is still on
            if (moving or driving) and not self._repeat_stops:
                self.trip(self.fault, self.fault_kind)
            return
        if not (self.motion_active or self.jog_active):
            return self.trip("motion command outside a motion block", "abort")
        if self.motion_active and not self.ch.deadman_held(self.deadman_enabled):
            return self.trip("deadman released", "pause")
        if moving:
            age = link.enc_age()
            if age is None or age > self.ctx.cfg["encoder"]["stale_s"]:
                return self.trip("no encoder data while steering", "abort")
            if self.move_t0 and now - self.move_t0 > self.max_move_s() + 0.3:
                return self.trip(f"steering moved longer than {self.max_move_s():.1f} s", "abort")
            stall = self.sc.get("stall_s", 0.4)
            last_change = max(link.enc_change_t or 0, self.move_t0 or 0)
            if self.move_t0 and now - self.move_t0 > stall + 0.3 and now - last_change > stall \
                    and not self._no_motion_noted:
                near_lock = True
                r = self.cal.get("range")
                if r and self.cal.zero_raw is not None and self.cal.ticks_per_deg():
                    pos = self.cal.pos(link.enc[1])
                    lock = r["lock_left_pos"] if self.move_dir == "left" else r["lock_right_pos"]
                    near_lock = abs(pos - lock) < 3 * self.cal.ticks_per_deg()
                hit = self.move_dir
                if near_lock:
                    link.send(steering=self.stop_value, note="stall stop")
                    self.note_steer("stop", now)
                    self.limit_hit = hit
                    self.ctx.rec.event("stall", direction=hit, raw=link.enc[1])
                    self.ch.log(f"steering {hit} stalled at the lock: stopped", "warn")
                else:   # most likely the command was dropped; nothing is moving
                    self._no_motion_noted = True
                    self.ctx.rec.event("no_motion", direction=hit, raw=link.enc[1])
                return
            lim = self.cal.soft_limits()
            if lim and self.cal.zero_raw is not None:
                pos = self.cal.pos(link.enc[1])
                lead = self.ctx.steer.stop_lead_ticks(self.move_dir) if self.ctx.steer else 0
                if (self.move_dir == "left" and pos + lead >= lim[1]) or \
                        (self.move_dir == "right" and pos - lead <= lim[0]):
                    hit = self.move_dir
                    link.send(steering=self.stop_value, note="soft limit")
                    self.note_steer("stop", now)
                    self.limit_hit = hit
                    self.ctx.rec.event("soft_limit", pos=pos, lead=lead, direction=hit)
                    self.ch.log(f"soft limit reached at pos {pos}", "warn")
        if driving:
            v = self.vn.speed_kmh()
            if v is not None and v > self.sc["max_speed_kmh"]:
                return self.trip(f"overspeed {v:.1f} km/h", "abort")
            if self.throttle_t0 and now - self.throttle_t0 > self.sc["max_throttle_run_s"]:
                return self.trip("throttle on too long", "abort")
