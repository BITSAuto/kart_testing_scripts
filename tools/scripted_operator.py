"""A scripted stand-in for the dashboard and the person using it, for the fake kart.

It runs a procedure with --tui, holds SPACE (sends 'hb' every 30 ms), answers prompts
(angles are read from the fake kart's true state), and does the jog steps by steering
toward the target the instruction names. It can also release SPACE, abort, or vanish
(close stdin) at a given time, to exercise the safety paths. Used by tools/self_test.py.
"""
import json
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run_procedure(script, args, config, state_path, timeout=180, release_at=None,
                  abort_at=None, close_at=None, answers=None):
    """Returns dict(returncode, result, run_dir, lines, events)."""
    state_path = Path(state_path)
    cmd = [sys.executable, "-u", str(ROOT / "procedures" / script), "--tui", "--config",
           str(config), *args]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, bufsize=1, cwd=str(ROOT))
    lock = threading.Lock()
    st = {"jog": None, "result": None, "run_dir": None, "closed": False}
    lines = []
    t0 = time.time()

    def send(s):
        with lock:
            if st["closed"]:
                return
            try:
                proc.stdin.write(s + "\n")
                proc.stdin.flush()
            except (BrokenPipeError, ValueError, OSError):
                pass

    def true_state():
        for _ in range(10):
            try:
                return json.loads(state_path.read_text())
            except (OSError, json.JSONDecodeError):
                time.sleep(0.01)
        return {"road_wheel_deg": 0.0}

    def heartbeat():
        while proc.poll() is None:
            el = time.time() - t0
            if abort_at is not None and el > abort_at:
                send("abort")
            if close_at is not None and el > close_at and not st["closed"]:
                with lock:
                    st["closed"] = True
                    proc.stdin.close()
            if not (release_at is not None and release_at < el < release_at + 1.5):
                send("hb")
            time.sleep(0.03)

    def jogger():
        while proc.poll() is None:
            tgt = st["jog"]
            if tgt is None:
                time.sleep(0.05)
                continue
            done_t = None
            while st["jog"] is tgt and proc.poll() is None:
                road = true_state()["road_wheel_deg"]
                err = tgt - road
                at_lock = abs(tgt) >= 15 and road * (1 if tgt > 0 else -1) >= 14.4
                if (abs(tgt) < 15 and abs(err) > 2.0) or (abs(tgt) >= 15 and not at_lock):
                    send("jog left" if err > 0 else "jog right")
                    done_t = None
                else:
                    done_t = done_t or time.time()
                    if time.time() - done_t > 1.0:
                        send("key enter")
                        while st["jog"] is tgt and proc.poll() is None:
                            time.sleep(0.02)
                time.sleep(0.03)

    threading.Thread(target=heartbeat, daemon=True).start()
    threading.Thread(target=jogger, daemon=True).start()
    killer = threading.Timer(timeout, lambda: proc.kill())
    killer.start()
    try:
        for line in proc.stdout:
            line = line.rstrip("\n")
            if not line.startswith("@"):
                lines.append(line)
                continue
            try:
                m = json.loads(line[1:])
            except json.JSONDecodeError:
                continue
            k = m.get("k")
            if k == "prompt":
                s = true_state()
                if answers and m["text"] in answers:
                    ans = answers[m["text"]]
                elif m["kind"] == "confirm":
                    ans = "y"
                elif m["kind"] == "number":
                    deg = s.get("road_wheel_deg", 0.0)
                    ans = f"{deg * (0.92 if 'RIGHT front' in m['text'] else 1.0):.2f}"
                else:
                    ans = ""
                send(f"answer {m['id']} {ans}")
            elif k == "status" and "jog" in m:
                if m["jog"]:
                    t = m.get("text", "")
                    st["jog"] = 0.0 if "STRAIGHT" in t else (15.0 if "LEFT" in t else -15.0)
                else:
                    st["jog"] = None
            elif k == "result":
                st["result"] = m
            elif k == "run_dir":
                st["run_dir"] = m["path"]
    finally:
        killer.cancel()
    proc.wait()
    events = []
    if st["run_dir"]:
        ev = Path(st["run_dir"]) / "events.csv"
        if ev.exists():
            events = [ln.split(",")[2] for ln in ev.read_text().splitlines()[1:] if ln.count(",") >= 2]
    return {"returncode": proc.returncode, "result": st["result"], "run_dir": st["run_dir"],
            "lines": lines, "events": events, "timed_out": time.time() - t0 >= timeout}
