"""Small numeric helpers used by the procedures."""
import math

import numpy as np


def stats(values):
    v = np.array([x for x in values if x is not None and not math.isnan(x)], float)
    if len(v) == 0:
        return {"n": 0}
    return {"n": int(len(v)), "mean": float(v.mean()), "std": float(v.std(ddof=1)) if len(v) > 1 else 0.0,
            "min": float(v.min()), "max": float(v.max()), "median": float(np.median(v)),
            "p95": float(np.percentile(v, 95))}


def linfit(x, y):
    """slope, intercept, r^2"""
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 2 or np.ptp(x) == 0:
        return None, None, None
    a, b = np.polyfit(x, y, 1)
    pred = a * x + b
    ss = ((y - y.mean()) ** 2).sum()
    r2 = 1 - ((y - pred) ** 2).sum() / ss if ss > 0 else 1.0
    return float(a), float(b), float(r2)


def series(link, cal, t0, t1=None):
    """(t, pos) numpy arrays of encoder samples between t0 and t1."""
    s = link.samples_between(t0, t1)
    if not s:
        return np.array([]), np.array([])
    t = np.array([a for a, _ in s])
    p = np.array([cal.pos(b) for _, b in s], float)
    return t, p


def first_change(t, p, t_after, p_ref, noise):
    """Time of the first sample after t_after that differs from p_ref by more than noise."""
    idx = np.where((t > t_after) & (np.abs(p - p_ref) > noise))[0]
    return (float(t[idx[0]]), int(idx[0])) if len(idx) else (None, None)


def settle_time(t, p, t_after, noise):
    """Time of the last sample after t_after that is more than noise away from the final value."""
    m = t > t_after
    if m.sum() == 0:
        return None
    tt, pp = t[m], p[m]
    final = pp[-1]
    idx = np.where(np.abs(pp - final) > noise)[0]
    if len(idx) == 0:
        return float(tt[0])
    return float(tt[min(idx[-1] + 1, len(tt) - 1)])


def steady_rate(t, p, frac=(0.2, 0.8)):
    """Rate from a line fit over the middle of a movement (by travelled distance)."""
    if len(t) < 4:
        return None, None
    travel = np.abs(p - p[0])
    total = travel[-1] if travel[-1] else travel.max()
    if total == 0:
        return 0.0, None
    m = (travel >= frac[0] * total) & (travel <= frac[1] * total)
    if m.sum() < 3:
        m = np.ones_like(t, bool)
    a, _, r2 = linfit(t[m], p[m])
    return a, r2


def circle_fit(x, y):
    """Least-squares circle (Kasa). Returns cx, cy, r, rms residual."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 5:
        return None
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x ** 2 + y ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = sol
    r = math.sqrt(max(c + cx ** 2 + cy ** 2, 0))
    rms = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2)))
    return float(cx), float(cy), float(r), rms


def clean(obj):
    """Make numpy values JSON friendly and round floats."""
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    if isinstance(obj, (np.floating, float)):
        return None if math.isnan(obj) else round(float(obj), 5)
    if isinstance(obj, np.integer):
        return int(obj)
    return obj
