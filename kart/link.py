"""Serial link to the drive-by-wire ESP32: sends frames, reads lines, parses the encoder.

Every frame sent and every line received is timestamped with time.monotonic() at the moment
it leaves / fully arrives, and logged by the Recorder. Encoder samples are also kept in memory
(self.samples, a list of (t, raw)) for the procedures to analyse.
"""
import re
import threading
import time

import serial

from .protocol import Command


class DbwLink:
    def __init__(self, cfg, rec=None, log=print):
        self.cfg, self.rec, self.log = cfg, rec, log
        s = cfg["serial"]
        self.port, self.baud = s["port"], int(s["baud"])
        self.eol = s.get("line_ending", "").encode()
        self.poll_s = float(s.get("poll_interval_s", 0) or 0)
        e = cfg["encoder"]
        self.enc_re = re.compile(e["regex"])
        self.enc_port = e.get("port") or ""
        self.cmd = Command(a=cfg["frame"]["a"], c=cfg["frame"]["c"],
                           steering=cfg["steering"]["stop"])
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.samples = []            # (t, raw) encoder samples
        self.enc = (None, None)      # latest (t, raw)
        self.enc_change_t = None     # last time the count changed
        self.rx_count = self.tx_count = self.enc_count = 0
        self.last_rx_t = self.last_tx_t = self.last_steer_tx_t = self.last_cmd_tx_t = None
        self.last_lines = []         # recent raw lines, for the probe and the dashboard
        self.fault = None
        self.ser = self.enc_ser = None
        self._threads = []

    # ----------------------------------------------------------------- open/close
    def open(self, send_neutral=False):
        kw = dict(bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                  stopbits=serial.STOPBITS_ONE, timeout=0.02, write_timeout=0.5)
        excl = bool(self.cfg["serial"].get("exclusive", True))
        self.ser = serial.Serial(self.port, self.baud, exclusive=excl, **kw)
        self._spawn(self._reader, self.ser, "dbw")
        if self.enc_port and self.enc_port != self.port:
            self.enc_ser = serial.Serial(self.enc_port, int(self.cfg["encoder"]["baud"]),
                                         exclusive=excl, **kw)
            self._spawn(self._reader, self.enc_ser, "enc")
        if self.poll_s > 0:
            self._spawn(self._poller)
        if send_neutral:
            self.send(note="neutral on open")

    def _spawn(self, fn, *args):
        th = threading.Thread(target=fn, args=args, daemon=True)
        th.start()
        self._threads.append(th)

    def close(self):
        self._stop.set()
        for th in self._threads:
            th.join(timeout=0.5)
        for s in (self.ser, self.enc_ser):
            try:
                if s:
                    s.close()
            except Exception:
                pass

    # ----------------------------------------------------------------------- send
    def send(self, cmd: Command = None, note="", **changes) -> float:
        """Send a frame (the last one with `changes` applied by default). Returns its time."""
        with self._lock:
            new = (cmd or self.cmd).with_(**changes) if changes else (cmd or self.cmd)
            frame = new.frame()
            try:
                self.ser.write(frame.encode() + self.eol)
                self.ser.flush()
            except (serial.SerialException, OSError) as e:
                self.fault = f"serial write failed: {e}"
                raise
            t = time.monotonic()
            if new.steering != self.cmd.steering:
                self.last_steer_tx_t = t
            self.cmd, self.last_tx_t = new, t
            if note != "poll":
                self.last_cmd_tx_t = t
            self.tx_count += 1
        if self.rec:
            self.rec.tx(t, frame, new, note)
        return t

    def send_reliable(self, repeats=3, spacing=0.25, note="", **changes):
        """Send the same state several times, spaced so at least one lands outside the window
        in which the kart may drop a command. For stops, throttle 0 and brake."""
        t = self.send(note=note, **changes)
        for i in range(1, repeats):
            time.sleep(spacing)
            self.send(note=f"{note} (repeat {i})")
        return t

    def _poller(self):
        while not self._stop.wait(self.poll_s):
            if self.last_tx_t is None or time.monotonic() - self.last_tx_t >= self.poll_s:
                try:
                    self.send(note="poll")
                except Exception:
                    return

    # ----------------------------------------------------------------------- read
    def _reader(self, ser, source):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError) as e:
                if not self._stop.is_set():
                    self.fault = f"serial read failed ({source}): {e}"
                return
            if not chunk:
                continue
            t = time.monotonic()
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle(t, source, line.decode(errors="replace").strip())
            if len(buf) > 4096:          # no newlines at all: keep it from growing forever
                self._handle(t, source, buf.decode(errors="replace").strip())
                buf = b""

    def _handle(self, t, source, text):
        if not text:
            return
        self.rx_count += 1
        self.last_rx_t = t
        self.last_lines.append((t, source, text))
        if len(self.last_lines) > 200:
            del self.last_lines[:100]
        raw = None
        if source == "enc" or not self.enc_port or self.enc_port == self.port:
            m = self.enc_re.search(text)
            if m:
                try:
                    raw = int(m.group(1))
                except (ValueError, IndexError):
                    raw = None
        if raw is not None:
            if raw != self.enc[1]:
                self.enc_change_t = t
            self.enc = (t, raw)
            self.samples.append((t, raw))
            self.enc_count += 1
        if self.rec:
            self.rec.rx(t, source, text, raw)

    # ------------------------------------------------------------------- queries
    def enc_age(self):
        t = self.enc[0]
        return None if t is None else time.monotonic() - t

    def samples_between(self, t0, t1=None):
        out = []
        for t, raw in reversed(self.samples):
            if t < t0:
                break
            if t1 is None or t <= t1:
                out.append((t, raw))
        out.reverse()
        return out
