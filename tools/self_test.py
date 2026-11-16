#!/usr/bin/env python3
"""Self-test: is this laptop ready, and does the whole bench work end to end?

    .venv/bin/python tools/self_test.py            # quick (about 1 minute)
    .venv/bin/python tools/self_test.py --full     # every procedure on the fake kart (~4 min)

1. Environment: Python, packages, config, data folder.
2. Host: dialout membership (and whether it is active in this login), brltty, ModemManager,
   the kart udev rules, /dev/kart_dbw and /dev/kart_vectornav, the configured ports.
3. Fake kart (sim/fake_kart.py on private virtual ports, data in a temporary folder, so real
   data and a running --sim dashboard are untouched): probe, health check, then the safety
   paths: deadman released mid-trial, operator abort, dashboard vanishing mid-motion.
   --full also runs p02-p11 and checks the measurements against the fake kart's true values.
4. Dashboard: starts headless, receives live data, runs a health check from the UI.

Nothing here touches the real kart: no frame is sent to config.toml's port.
Report: data/self_test/<time>.json. Exit code 0 = no failures.
"""
import argparse
import asyncio
import grp
import json
import os
import pwd
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.scripted_operator import run_procedure  # noqa: E402

RESULTS = []
COLOR = {"PASS": "\033[32m", "WARN": "\033[33m", "FAIL": "\033[31m", "SKIP": "\033[90m"}


def report(section, name, status, detail=""):
    RESULTS.append({"section": section, "check": name, "status": status, "detail": detail})
    c = COLOR.get(status, "") if sys.stdout.isatty() else ""
    r = "\033[0m" if c else ""
    print(f"  {c}{status:4s}{r}  {name}: {detail}", flush=True)


# ------------------------------------------------------------------- 1. environment
def check_environment():
    print("\n[1] environment")
    v = sys.version_info
    report("env", "python", "PASS" if v >= (3, 11) else "FAIL", sys.version.split()[0])
    for mod in ("serial", "numpy", "textual"):
        try:
            m = __import__(mod)
            report("env", f"package {mod}", "PASS", getattr(m, "__version__", "ok"))
        except ImportError as e:
            report("env", f"package {mod}", "FAIL", str(e))
    try:
        from kart.config import load_config
        cfg = load_config()
        report("env", "config.toml", "PASS", cfg["_path"])
    except Exception as e:
        report("env", "config.toml", "FAIL", str(e))
        return None
    root = Path(cfg["_data_root"])
    root.mkdir(parents=True, exist_ok=True)
    ok = os.access(root, os.W_OK)
    free = shutil.disk_usage(root).free / 1e9
    report("env", "data folder", "PASS" if ok and free > 1 else ("WARN" if ok else "FAIL"),
           f"{root} ({free:.1f} GB free)")
    return cfg


# ------------------------------------------------------------------------- 2. host
def check_host(cfg):
    print("\n[2] host serial setup")
    user = pwd.getpwuid(os.getuid()).pw_name
    try:
        members = grp.getgrnam("dialout").gr_mem
        gid = grp.getgrnam("dialout").gr_gid
    except KeyError:
        report("host", "dialout group", "FAIL", "no dialout group on this system")
        members, gid = [], None
    in_db = user in members or (gid is not None and pwd.getpwnam(user).pw_gid == gid)
    active = gid in os.getgroups() if gid is not None else False
    if active:
        report("host", "dialout", "PASS", f"{user} is in dialout (active in this login)")
    elif in_db:
        report("host", "dialout", "WARN", f"{user} was added to dialout, but this login doesn't "
               "have it yet: log out and back in (or reboot)")
    else:
        report("host", "dialout", "FAIL", f"{user} not in dialout: ./setup.sh (or sudo usermod "
               "-aG dialout $USER)")

    brl_rule = Path("/usr/lib/udev/rules.d/85-brltty.rules")
    masked = Path("/etc/udev/rules.d/85-brltty.rules")
    if not brl_rule.exists() and not Path("/lib/udev/rules.d/85-brltty.rules").exists():
        report("host", "brltty", "PASS", "not installed")
    elif masked.exists() and os.path.realpath(masked) == "/dev/null":
        report("host", "brltty", "PASS", "installed, USB-serial grab disabled (udev rule masked)")
    else:
        report("host", "brltty", "FAIL", "installed and can grab CH340/CP210x ports (they "
               "vanish right after plugging in): ./setup.sh fixes it")

    mm = subprocess.run(["systemctl", "is-active", "ModemManager"], capture_output=True, text=True)
    rules = Path("/etc/udev/rules.d/70-kart-serial.rules")
    if mm.stdout.strip() != "active":
        report("host", "ModemManager", "PASS", "not running")
    elif rules.exists() and "ID_MM_DEVICE_IGNORE" in rules.read_text():
        report("host", "ModemManager", "PASS", "running, kart ports excluded by udev rule")
    else:
        report("host", "ModemManager", "WARN", "running and may probe the ESP32 with AT commands: "
               "./setup.sh installs the ignore rule")
    if rules.exists():
        txt = rules.read_text()
        names = [n for n in ("kart_dbw", "kart_vectornav") if n in txt]
        report("host", "udev rules", "PASS" if names else "WARN",
               f"{rules}" + (f", names: {', '.join(names)}" if names else
                             " (generic only; run ./setup.sh --udev with the kart plugged in)"))
    else:
        report("host", "udev rules", "WARN", "not installed: ./setup.sh")
    for alias in ("/dev/kart_dbw", "/dev/kart_vectornav"):
        if os.path.exists(alias):
            report("host", alias, "PASS", f"-> {os.path.realpath(alias)}")
        else:
            report("host", alias, "SKIP", "not present (device unplugged or no udev name yet)")
    if cfg:
        port = cfg["serial"]["port"]
        if os.path.exists(port):
            okrw = os.access(os.path.realpath(port), os.R_OK | os.W_OK)
            report("host", "configured drive-by-wire port", "PASS" if okrw else "FAIL",
                   f"{port} {'readable/writable' if okrw else 'NO PERMISSION'}")
        else:
            report("host", "configured drive-by-wire port", "WARN",
                   f"{port} not present (kart not connected?)")
        if cfg["vectornav"]["source"] == "serial":
            vp = cfg["vectornav"]["port"]
            report("host", "configured VectorNav port", "PASS" if os.path.exists(vp) else "WARN",
                   vp + ("" if os.path.exists(vp) else " not present"))


# -------------------------------------------------------------------- 3. fake kart
class FakeKart:
    def __init__(self, tmp):
        self.tmp = Path(tmp)
        self.run = self.tmp / "run"
        self.data = self.tmp / "data"
        cfg_text = (ROOT / "sim" / "sim_config.toml").read_text()
        cfg_text = cfg_text.replace('port = "sim/run/dbw"', f'port = "{self.run / "dbw"}"')
        cfg_text = cfg_text.replace('port = "sim/run/vn"', f'port = "{self.run / "vn"}"')
        cfg_text = cfg_text.replace('root = "sim/data"', f'root = "{self.data}"')
        self.config = self.tmp / "self_test_config.toml"
        self.config.write_text(cfg_text)
        self.proc = subprocess.Popen([sys.executable, str(ROOT / "sim" / "fake_kart.py"), "--vn",
                                      "--seed", "7", "--run-dir", str(self.run)],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        for _ in range(50):
            if (self.run / "dbw").exists() and (self.run / "state.json").exists():
                break
            time.sleep(0.1)
        self.state = self.run / "state.json"

    def cal(self, name):
        p = self.data / "calibration" / f"{name}.json"
        return json.loads(p.read_text()) if p.exists() else None

    def stop(self):
        self.proc.terminate()
        try:
            self.proc.wait(3)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def last_frames(run_dir, n=3):
    lines = (Path(run_dir) / "serial_tx.csv").read_text().splitlines()[-n:]
    return [ln.split(",")[2] for ln in lines]


def run_and_check(fk, name, script, args, check=None, **kw):
    r = run_procedure(script, args, fk.config, fk.state, **kw)
    status = (r["result"] or {}).get("status")
    if r["timed_out"]:
        report("sim", name, "FAIL", "timed out; last output: " + " | ".join(r["lines"][-3:]))
        return r
    if check is None:
        ok = status == "completed"
        report("sim", name, "PASS" if ok else "FAIL",
               status or "no result" + (" | " + " | ".join(r["lines"][-3:]) if not ok else ""))
    else:
        try:
            check(r)
        except AssertionError as e:
            report("sim", name, "FAIL", str(e) or "check failed")
    return r


def check_sim(full):
    print("\n[3] fake kart (" + ("full" if full else "quick") + ")")
    tmp = tempfile.mkdtemp(prefix="kart_self_test_")
    fk = FakeKart(tmp)
    try:
        if not (fk.run / "dbw").exists():
            report("sim", "fake kart", "FAIL", "did not start")
            return
        report("sim", "fake kart", "PASS", f"ports in {fk.run}")
        run_and_check(fk, "p00 serial probe", "p00_serial_probe.py", ["--listen", "1"])

        def health(r):
            s = (r["result"] or {}).get("summary") or {}
            fails = [c["name"] for c in s.get("checks", []) if c["status"] == "fail"]
            assert not fails, f"health check failures: {fails}"
            report("sim", "p01 health check", "PASS", f"overall {s.get('overall')}")
        run_and_check(fk, "p01 health check", "p01_health_check.py", ["--listen", "1"], health)

        def p02(r):
            m = fk.cal("steer_map")
            assert m, "no steer_map written"
            assert m["left_value"] < 50 < m["right_value"], f"left/right wrong: {m}"
            assert m["mode"] == "constant", f"mode {m['mode']}"
            report("sim", "p02 steering map", "PASS",
                   f"left D{m['left_value']}, right D{m['right_value']}, {m['mode']}")
        run_and_check(fk, "p02 steering map", "p02_steer_command_map.py",
                      ["--values", "30,70,0,100"], p02)

        def p03(r):
            g = fk.cal("range")
            assert g and not g.get("provisional"), "no range written"
            assert 100 < g["ticks_per_deg"] < 145, f"ticks/deg {g['ticks_per_deg']} (true 120)"
            report("sim", "p03 centre/locks/table", "PASS",
                   f"{g['ticks_per_deg']:.0f} ticks/deg (true 120 + backlash), locks "
                   f"{g['lock_left_deg']:+.1f}/{g['lock_right_deg']:+.1f} deg")
        run_and_check(fk, "p03 centre/locks/table", "p03_center_and_range.py", ["--points", "5"], p03)

        def p04(r):
            s = fk.cal("slew")
            assert s, "no slew written"
            L, R = s["left"]["rate_ticks_s"], s["right"]["rate_ticks_s"]
            assert abs(L - 1440) < 150 and abs(R - 1080) < 120, f"rates {L:.0f}/{R:.0f} (true 1440/1080)"
            report("sim", "p04 slew rate", "PASS", f"{L:.0f} / {R:.0f} ticks/s (true 1440 / 1080)")
        run_and_check(fk, "p04 slew rate", "p04_slew_rate.py", ["--reps", "1"], p04)

        # ---- safety paths
        def deadman(r):
            assert "trial_interrupted" in r["events"], "deadman release did not interrupt a trial"
            assert (r["result"] or {}).get("status") == "completed", "did not finish after resume"
            report("sim", "deadman release -> stop, redo trial", "PASS", "interrupted and redone")
        run_and_check(fk, "deadman release", "p04_slew_rate.py", ["--reps", "2"], deadman,
                      release_at=3.0)

        def abort(r):
            assert (r["result"] or {}).get("status") == "aborted", f"status {(r['result'] or {}).get('status')}"
            fr = last_frames(r["run_dir"])
            assert all("D50" in f and "B0" in f for f in fr), f"last frames {fr}"
            assert "safety_stop" in r["events"], "no safety_stop event"
            report("sim", "abort -> stop frames", "PASS", "aborted, last frames D50 B0")
        run_and_check(fk, "operator abort", "p04_slew_rate.py", ["--reps", "3"], abort, abort_at=4.0)

        def vanish(r):
            fr = last_frames(r["run_dir"])
            assert all("D50" in f and "B0" in f for f in fr), f"last frames {fr}"
            st = json.loads(fk.state.read_text())
            assert st["D"] == 50 and st["B"] == 0, f"fake kart left at D{st['D']} B{st['B']}"
            report("sim", "dashboard vanishes -> stop", "PASS", "kart stopped (D50 B0)")
        run_and_check(fk, "dashboard vanishes", "p04_slew_rate.py", ["--reps", "3"], vanish,
                      close_at=4.0)

        if full:
            def p05(r):
                lat = fk.cal("latency")
                d = lat["left"]["dead_time_s"]
                assert 0.12 < d < 0.32, f"dead time {d}"
                report("sim", "p05 delay/overshoot", "PASS", f"delay {d:.3f} s (true ~0.20 + "
                       "line time)")
            run_and_check(fk, "p05 delay/overshoot", "p05_latency_overshoot.py", ["--reps", "3"], p05)

            def p06(r):
                d = fk.cal("drop")
                by = {x["gap_s"]: x for x in d["by_gap"]}
                assert by[0.4]["dropped"] == 0, "drops at 0.4 s (fake drops only under 0.2 s)"
                report("sim", "p06 dropped commands", "PASS",
                       ", ".join(f"{g}s: {x['dropped']}/{x['n']}" for g, x in by.items()))
            run_and_check(fk, "p06 dropped commands", "p06_command_drop.py",
                          ["--spacings", "0.05,0.4", "--reps", "4"], p06)
            run_and_check(fk, "p07 backlash", "p07_backlash.py", ["--positions", "0", "--reps", "1"])
            run_and_check(fk, "p08 centre repeatability", "p08_center_repeatability.py",
                          ["--trials", "2"])

            def p09(r):
                lo = fk.cal("longitudinal")
                v = lo["by_throttle"][0]["steady_kmh"]
                assert abs(v - 6.0) < 0.6, f"steady {v} km/h at throttle 60 (true 6.0)"
                report("sim", "p09 throttle/speed", "PASS", f"throttle 60 -> {v:.1f} km/h (true 6.0)")
            run_and_check(fk, "p09 throttle/speed", "p09_throttle_speed.py",
                          ["--throttle", "60", "--then", "brake"], p09)

            def p10(r):
                c = fk.cal("circles")["circles"][0]
                assert 5.5 < c["radius_fit_m"] < 8.5, f"radius {c['radius_fit_m']}"
                report("sim", "p10 lock circles", "PASS", f"radius {c['radius_fit_m']:.2f} m")
            run_and_check(fk, "p10 lock circles", "p10_lock_circles.py",
                          ["--throttle", "60", "--sides", "left", "--turns", "1"], p10)
            run_and_check(fk, "p11 system-ID drive", "p11_sysid_drive.py", ["--duration", "10"])
    finally:
        fk.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# --------------------------------------------------------------------- 4. dashboard
def check_dashboard():
    print("\n[4] dashboard (headless)")
    try:
        from dashboard.app import KartDashboard
    except Exception as e:
        report("ui", "dashboard import", "FAIL", str(e))
        return
    tmp = tempfile.mkdtemp(prefix="kart_ui_test_")
    fk = FakeKart(tmp)
    out = {}

    async def go():
        app = KartDashboard(fk.config)
        async with app.run_test(size=(160, 44)) as pilot:
            for _ in range(60):
                await pilot.pause(0.1)
                if app.tel.get("raw") is not None:
                    break
            out["tel"] = app.tel.get("raw") is not None
            await pilot.press("h")
            for _ in range(150):
                await pilot.pause(0.1)
                if app.proc is None and app.query_one("#health_table").row_count:
                    break
            out["health_rows"] = app.query_one("#health_table").row_count
            await app.shutdown()
    try:
        asyncio.run(asyncio.wait_for(go(), 60))
        report("ui", "dashboard starts, live data", "PASS" if out.get("tel") else "FAIL",
               "telemetry received" if out.get("tel") else "no telemetry from the monitor")
        report("ui", "health check from the UI", "PASS" if out.get("health_rows") else "FAIL",
               f"{out.get('health_rows', 0)} checks shown")
    except Exception as e:
        report("ui", "dashboard", "FAIL", f"{type(e).__name__}: {e}")
    finally:
        fk.stop()
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--full", action="store_true", help="run every procedure on the fake kart")
    ap.add_argument("--skip-host", action="store_true")
    ap.add_argument("--skip-sim", action="store_true")
    ap.add_argument("--skip-ui", action="store_true")
    a = ap.parse_args()
    t0 = time.time()
    cfg = check_environment()
    if not a.skip_host:
        check_host(cfg)
    if not a.skip_sim:
        check_sim(a.full)
    if not a.skip_ui:
        check_dashboard()
    n = {s: sum(r["status"] == s for r in RESULTS) for s in ("PASS", "WARN", "FAIL", "SKIP")}
    print(f"\nself-test: {n['PASS']} passed, {n['WARN']} warnings, {n['FAIL']} failed, "
          f"{n['SKIP']} skipped ({time.time() - t0:.0f} s)")
    if cfg:
        d = Path(cfg["_data_root"]) / "self_test"
        d.mkdir(parents=True, exist_ok=True)
        p = d / f"{time.strftime('%Y-%m-%d_%H-%M-%S')}.json"
        p.write_text(json.dumps({"full": a.full, "counts": n, "results": RESULTS}, indent=2))
        print(f"report: {p}")
    sys.exit(1 if n["FAIL"] else 0)


if __name__ == "__main__":
    main()
