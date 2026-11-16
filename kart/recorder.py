"""One run = one directory: data/<test>/<YYYY-MM-DD_HH-MM-SS>[_label]/

    meta.json        test, arguments, config snapshot, calibration used, status, timings
    serial_tx.csv    every frame sent           (t, wall, frame, fields..., note)
    serial_rx.csv    every line received        (t, wall, source, enc_raw, line)
    vectornav.csv    VectorNav samples, if any  (t, wall, speed_mps, x, y, yaw_deg, ...)
    events.csv       trial starts/ends, prompts, operator answers, safety stops
    <test tables>    per-test results (trials.csv, sweeps.csv, ...)
    summary.json     results computed at the end

t is seconds since the run started (monotonic clock), wall is Unix time.
"""
import csv
import json
import re
import threading
import time
from pathlib import Path

from .protocol import FIELDS

VN_FIELDS = ["speed_mps", "x_m", "y_m", "yaw_deg", "yaw_rate", "lat", "lon", "fix", "source"]


def _slug(s):
    return re.sub(r"[^A-Za-z0-9_.-]+", "-", s).strip("-")[:40]


class Recorder:
    def __init__(self, data_root, test, label="", meta=None):
        stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        name = stamp + (f"_{_slug(label)}" if label else "")
        self.dir = Path(data_root) / test / name
        n = 1
        while self.dir.exists():
            self.dir = Path(data_root) / test / f"{name}_{n}"
            n += 1
        self.dir.mkdir(parents=True)
        self.t0 = time.monotonic()
        self.wall0 = time.time()
        self._lock = threading.Lock()
        self._files, self._writers = {}, {}
        self.meta = {"test": test, "label": label, "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                     "status": "running", **(meta or {})}
        self._write_meta()
        self.table("serial_tx", ["t", "wall", "frame", *FIELDS, "note"])
        self.table("serial_rx", ["t", "wall", "source", "enc_raw", "line"])
        self.table("events", ["t", "wall", "kind", "data"])

    def rel(self, t_mono):
        return round(t_mono - self.t0, 4)

    def _wall(self, t_mono):
        return round(self.wall0 + (t_mono - self.t0), 4)

    def table(self, name, header):
        with self._lock:
            if name not in self._writers:
                f = open(self.dir / f"{name}.csv", "w", newline="", buffering=1)
                w = csv.writer(f)
                w.writerow(header)
                self._files[name], self._writers[name] = f, w

    def row(self, name, values):
        with self._lock:
            w = self._writers.get(name)
            if w:
                w.writerow(values)

    # ---------------------------------------------------------- shortcuts
    def tx(self, t, frame, cmd, note=""):
        self.row("serial_tx", [self.rel(t), self._wall(t), frame,
                               *[getattr(cmd, f) for f in FIELDS], note])

    def rx(self, t, source, line, enc_raw):
        self.row("serial_rx", [self.rel(t), self._wall(t), source,
                               "" if enc_raw is None else enc_raw, line])

    def vn(self, t, state):
        if "vectornav" not in self._writers:
            self.table("vectornav", ["t", "wall", *VN_FIELDS])
        self.row("vectornav", [self.rel(t), self._wall(t),
                               *["" if state.get(k) is None else state.get(k) for k in VN_FIELDS]])

    def event(self, _kind, **data):
        t = time.monotonic()
        self.row("events", [self.rel(t), self._wall(t), _kind, json.dumps(data, default=str)])

    def write_json(self, name, obj):
        (self.dir / name).write_text(json.dumps(obj, indent=2, default=str))

    def _write_meta(self):
        (self.dir / "meta.json").write_text(json.dumps(self.meta, indent=2, default=str))

    def finish(self, status, summary=None):
        self.meta.update(status=status, ended=time.strftime("%Y-%m-%d %H:%M:%S"),
                         duration_s=round(time.monotonic() - self.t0, 2))
        if summary is not None:
            self.write_json("summary.json", summary)
        self._write_meta()
        with self._lock:
            for f in self._files.values():
                f.close()
            self._writers.clear()
