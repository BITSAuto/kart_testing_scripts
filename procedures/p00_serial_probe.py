#!/usr/bin/env python3
"""p00 serial probe: what does the ESP32 send?

1. listens without sending anything        -> does it stream on its own?
2. sends one neutral frame (D=stop, B=0)    -> does it answer frames? how fast?
3. optional --pulse D: a short steering pulse, to see which number in which line changes
   (that's the encoder) and suggest config.toml [encoder] regex.

Results: data/00_serial_probe/<run>/summary.json, shapes.csv (one row per line format).
"""
import csv
import re
import sys
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.harness import run, run_trial  # noqa: E402

NUM = re.compile(r"-?\d+(?:\.\d+)?")


def shape_of(line):
    return NUM.sub("#", line)


def regex_for(example, field):
    """Regex matching lines shaped like `example`, capturing number #field."""
    parts, last, out = list(NUM.finditer(example)), 0, ""
    for i, m in enumerate(parts):
        out += re.escape(example[last:m.start()])
        out += r"(-?\d+)" if i == field else r"-?\d+(?:\.\d+)?"
        last = m.end()
    return out + re.escape(example[last:])


def add_args(ap):
    ap.add_argument("--listen", type=float, default=3.0, help="seconds to listen in each phase")
    ap.add_argument("--no-send", action="store_true", help="never send anything")
    ap.add_argument("--pulse", type=int, default=None,
                    help="D value for a short steering pulse (e.g. 40), to find the encoder field")
    ap.add_argument("--pulse-s", type=float, default=0.4)


def body(ctx):
    a, link, rec = ctx.args, ctx.link, ctx.rec
    s = {}
    ctx.set_phase("listening, nothing sent")
    n0 = link.rx_count
    ctx.sleep(a.listen)
    s["lines_per_s_passive"] = (link.rx_count - n0) / a.listen
    ctx.log(f"passive: {s['lines_per_s_passive']:.1f} lines/s")

    if not a.no_send:
        ctx.set_phase("sent one neutral frame, listening")
        n1 = link.rx_count
        t1 = link.send(note="probe neutral frame")
        ctx.sleep(a.listen)
        s["lines_per_s_after_frame"] = (link.rx_count - n1) / a.listen
        first = [t for t, _, _ in link.last_lines if t > t1]
        s["first_line_after_frame_s"] = round(first[0] - t1, 4) if first else None
        ctx.log(f"after a frame: {s['lines_per_s_after_frame']:.1f} lines/s, first line "
                f"{s['first_line_after_frame_s']} s after sending")

    pulse_window = None
    if a.pulse is not None:
        ctx.set_phase(f"steering pulse D={a.pulse}")

        def pulse():
            t0 = ctx.steer.raw_command(a.pulse, note="probe pulse")
            ctx.sleep(min(a.pulse_s, ctx.cfg["safety"]["uncalibrated_pulse_s"]))
            ctx.steer.stop(note="probe pulse stop")
            return t0, time.monotonic()
        pulse_window = run_trial(ctx, f"probe pulse D={a.pulse}", pulse)
        ctx.sleep(0.5)

    # ---- analyse every line received (from the recording)
    ctx.set_phase("analysing")
    rows = list(csv.DictReader(open(rec.dir / "serial_rx.csv")))
    shapes = defaultdict(lambda: {"count": 0, "example": "", "values": []})
    for r in rows:
        line = r["line"]
        sh = shapes[shape_of(line)]
        sh["count"] += 1
        sh["example"] = sh["example"] or line
        sh["values"].append((float(r["t"]), [float(v) for v in NUM.findall(line)]))
    total = len(rows)
    out, best = [], None
    t_rel = (lambda tm: tm - rec.t0)
    for shape, d in sorted(shapes.items(), key=lambda kv: -kv[1]["count"]):
        nf = max((len(v) for _, v in d["values"]), default=0)
        fields = []
        for i in range(nf):
            vals = [(t, v[i]) for t, v in d["values"] if len(v) > i]
            nums = [v for _, v in vals]
            info = {"field": i, "min": min(nums), "max": max(nums),
                    "distinct": len(set(nums)), "integer": all(float(x).is_integer() for x in nums)}
            if pulse_window:
                p0, p1 = t_rel(pulse_window[0]), t_rel(pulse_window[1])
                before = [v for t, v in vals if t < p0]
                after = [v for t, v in vals if t > p1 + 0.2]
                if before and after:
                    info["change_during_pulse"] = after[-1] - before[-1]
                    if info["integer"] and abs(info["change_during_pulse"]) > 2 and (
                            best is None or abs(info["change_during_pulse"]) > abs(best[2])):
                        best = (shape, i, info["change_during_pulse"], d["example"])
            fields.append(info)
        out.append({"shape": shape, "count": d["count"], "share": d["count"] / max(total, 1),
                    "example": d["example"], "fields": fields})
    with open(rec.dir / "shapes.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["count", "share", "shape", "example", "fields"])
        for o in out:
            w.writerow([o["count"], round(o["share"], 3), o["shape"], o["example"], o["fields"]])

    for o in out[:8]:
        ctx.log(f"{o['count']:6d}x  {o['example'][:70]}")
    s["line_formats"] = out[:10]
    s["total_lines"] = total
    s["encoder_regex"] = ctx.cfg["encoder"]["regex"]
    s["encoder_lines_matched"] = link.enc_count
    s["encoder_match_share"] = link.enc_count / max(total, 1)
    sugg = []
    if s["lines_per_s_passive"] == 0 and s.get("lines_per_s_after_frame", 0) > 0:
        sugg.append("The ESP32 only answers frames: set [serial] poll_interval_s = 0.1")
    if total == 0:
        sugg.append("Nothing received: check the port, baud rate, cable and that the ESP32 is on.")
    if best:
        rx = regex_for(best[3], best[1])
        s["encoder_field_found"] = {"shape": best[0], "field": best[1], "change": best[2],
                                    "suggested_regex": rx}
        sugg.append(f"Encoder looks like field {best[1]} of '{best[3]}'. Suggested "
                    f"[encoder] regex = '{rx}'")
    elif link.enc_count == 0 and total:
        sugg.append("The encoder regex matched nothing. Run with --pulse 40 (holding SPACE) so "
                    "the probe can see which number moves.")
    s["suggestions"] = sugg
    for x in sugg:
        ctx.log(x, "step")
    return s


if __name__ == "__main__":
    run("00_serial_probe", "What the ESP32 sends; suggests the encoder regex", body, add_args,
        motion=lambda a: a.pulse is not None, send_neutral=False)
