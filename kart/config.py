"""Configuration (config.toml) and calibration results (data/calibration/*.json)."""
import json
import shutil
import time
import tomllib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config.toml"

# Calibration files written by the procedures, in the order they are produced.
CALIBRATIONS = {
    "steer_map": "p02: which D values steer left/right, and the encoder sign",
    "zero": "p03: raw encoder count with the wheels straight (redo every power cycle)",
    "range": "p03: lock positions and the encoder-ticks -> wheel-angle table",
    "slew": "p04: steering rate in each direction",
    "slew_rolling": "p04 --throttle: steering rate while rolling",
    "latency": "p05: command delay, stop delay and overshoot",
    "drop": "p06: chance a command is dropped vs spacing",
    "backlash": "p07: play between steering wheel and road wheels",
    "repeatability": "p08: centre repeatability / encoder drift",
    "longitudinal": "p09: throttle -> speed, acceleration and braking",
    "circles": "p10: turning radius at full lock",
}


def load_config(path=None) -> dict:
    p = Path(path) if path else DEFAULT_CONFIG
    with open(p, "rb") as f:
        cfg = tomllib.load(f)
    root = Path(cfg.get("data", {}).get("root", "data"))
    if not root.is_absolute():
        root = ROOT / root
    # Relative port paths (the simulator's links) are relative to this folder.
    for sec in ("serial", "encoder", "vectornav"):
        port = cfg.get(sec, {}).get("port")
        if port and not port.startswith("/") and "/" in port:
            cfg[sec]["port"] = str(ROOT / port)
    cfg["_path"] = str(p)
    cfg["_data_root"] = str(root)
    return cfg


class Calibration:
    """Latest calibration results, with helpers to turn encoder ticks into wheel angle.

    Position convention: "pos" is encoder ticks relative to the straight-ahead zero, with
    LEFT positive (ROS convention: positive angle = turning left).
    """

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.dir = Path(cfg["_data_root"]) / "calibration"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.reload()

    def reload(self):
        self.data = {}
        for name in CALIBRATIONS:
            p = self.dir / f"{name}.json"
            if p.exists():
                try:
                    self.data[name] = json.loads(p.read_text())
                except json.JSONDecodeError:
                    pass

    def get(self, name):
        return self.data.get(name)

    def save(self, name: str, obj: dict, run_dir=None):
        obj = dict(obj)
        obj["saved_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        if run_dir:
            obj["source_run"] = str(run_dir)
        p = self.dir / f"{name}.json"
        if p.exists():
            hist = self.dir / "history"
            hist.mkdir(exist_ok=True)
            shutil.copy2(p, hist / f"{time.strftime('%Y-%m-%d_%H-%M-%S')}_{name}.json")
        p.write_text(json.dumps(obj, indent=2))
        self.data[name] = obj

    # ------------------------------------------------------------ steering map
    @property
    def left_value(self) -> int:
        m = self.get("steer_map")
        return int(m["left_value"]) if m else int(self.cfg["steering"]["left"])

    @property
    def right_value(self) -> int:
        m = self.get("steer_map")
        return int(m["right_value"]) if m else int(self.cfg["steering"]["right"])

    @property
    def stop_value(self) -> int:
        return int(self.cfg["steering"]["stop"])

    @property
    def sign(self) -> int:
        """+1 if steering left increases the raw count."""
        m = self.get("steer_map")
        return int(m.get("left_ticks_sign", 1)) if m else 1

    @property
    def zero_raw(self):
        z = self.get("zero")
        return z["raw_ticks"] if z else None

    def pos(self, raw):
        """Raw count -> position (ticks from straight ahead, left positive)."""
        if raw is None:
            return None
        return (raw - (self.zero_raw or 0)) * self.sign

    def raw(self, pos):
        return pos * self.sign + (self.zero_raw or 0)

    # ------------------------------------------------------------- angle map
    def deg(self, pos):
        r = self.get("range")
        if pos is None or not r or not r.get("deg_table"):
            return None
        t = np.array(r["deg_table"], float)
        t = t[np.argsort(t[:, 0])]
        if len(t) == 1:
            return pos / r["ticks_per_deg"]
        if pos < t[0, 0]:
            return t[0, 1] + (pos - t[0, 0]) / r["ticks_per_deg"]
        if pos > t[-1, 0]:
            return t[-1, 1] + (pos - t[-1, 0]) / r["ticks_per_deg"]
        return float(np.interp(pos, t[:, 0], t[:, 1]))

    def ticks_per_deg(self):
        r = self.get("range")
        return r["ticks_per_deg"] if r and r.get("ticks_per_deg") else None

    def soft_limits(self):
        """(min_pos, max_pos) inside the locks, or None before p03 has run."""
        r = self.get("range")
        if not r:
            return None
        margin = self.cfg["safety"]["soft_limit_margin_deg"] * (self.ticks_per_deg() or 0)
        lo, hi = sorted((r["lock_right_pos"], r["lock_left_pos"]))
        return lo + margin, hi - margin

    def has(self, *names):
        return all(n in self.data for n in names)
