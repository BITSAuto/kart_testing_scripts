#!/usr/bin/env python3
"""Kart test bench dashboard (terminal UI, no GPU needed).

    ./run_dashboard.sh                 # real kart, config.toml
    ./run_dashboard.sh --sim           # dry run: starts sim/fake_kart.py and uses sim/sim_config.toml
    ./run_dashboard.sh --config other.toml

Left: procedures (in run order) with their arguments. Right: live status, steering and speed
charts, track, and the procedure's own output. Tabs: Live / Health / Data.

Keys while a procedure runs:
    SPACE (hold)   deadman: anything that moves the kart needs it held; release = stop
    A / D (hold)   jog steering left / right (when a procedure asks you to jog)
    ENTER          "done" in jog steps
    W / S / B      throttle on / off, brake (manual drive)
    ESC or X       ABORT: stop, brake if rolling, end the procedure
Other keys: R run selected, H health check, M monitor on/off, C clear charts, Q quit.
"""
import argparse
import asyncio
import json
import os
import shlex
import signal
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rich.table import Table  # noqa: E402
from rich.text import Text  # noqa: E402
from textual import events  # noqa: E402
from textual.app import App, ComposeResult  # noqa: E402
from textual.binding import Binding  # noqa: E402
from textual.containers import Horizontal, Vertical, VerticalScroll  # noqa: E402
from textual.screen import ModalScreen  # noqa: E402
from textual.widgets import (Button, DataTable, DirectoryTree, Footer, Header, Input,  # noqa: E402
                             Label, ListItem, ListView, RichLog, Static, TabbedContent, TabPane)

from dashboard.plot import CommandStrip, LineChart, XYChart  # noqa: E402
from kart.config import CALIBRATIONS, Calibration, load_config  # noqa: E402
from procedures.registry import PROCEDURES  # noqa: E402

LEVEL_STYLE = {"[OK]": "green", "[WARN]": "yellow", "[ERROR]": "bold red", "==>": "bold cyan",
               "SAFETY": "bold red"}


# --------------------------------------------------------------------------- dialogs
class PromptScreen(ModalScreen):
    """Question from a procedure. ESC aborts the procedure."""
    BINDINGS = [Binding("escape", "cancel", "Abort procedure"),
                Binding("y", "yes", "Yes", show=False), Binding("n", "no", "No", show=False)]
    DEFAULT_CSS = """
    PromptScreen { align: center middle; }
    #dlg { width: 76; height: auto; border: thick $warning; background: $surface; padding: 1 2; }
    #q { margin-bottom: 1; }
    #btns { height: auto; }
    #btns Button { margin-right: 2; }
    """

    def __init__(self, msg):
        super().__init__()
        self.msg = msg

    def compose(self):
        m = self.msg
        with Vertical(id="dlg"):
            yield Static(Text(m["text"], style="bold"), id="q")
            if m["kind"] == "confirm":
                with Horizontal(id="btns"):
                    yield Button("Yes (y)", id="yes", variant="success")
                    yield Button("No (n)", id="no", variant="error")
            elif m["kind"] == "choice" and m.get("choices"):
                with Horizontal(id="btns"):
                    for c in m["choices"]:
                        yield Button(c, id=f"c_{c}")
            else:
                yield Input(placeholder="type and press ENTER" + (
                    f" (default {m['default']})" if m.get("default") not in (None, "") else ""),
                    type="number" if m["kind"] == "number" else "text", id="inp")
            yield Static(Text("ESC aborts the procedure", style="dim"))

    def on_mount(self):
        try:
            self.query_one("#inp", Input).focus()
        except Exception:
            self.set_focus(None)

    def on_input_submitted(self, ev: Input.Submitted):
        self.dismiss(ev.value.strip())

    def on_button_pressed(self, ev: Button.Pressed):
        bid = ev.button.id or ""
        self.dismiss("y" if bid == "yes" else "n" if bid == "no" else bid[2:])

    def action_yes(self):
        if self.msg["kind"] == "confirm":
            self.dismiss("y")

    def action_no(self):
        if self.msg["kind"] == "confirm":
            self.dismiss("n")

    def action_cancel(self):
        self.dismiss("__abort__")


class ConfirmQuit(ModalScreen):
    BINDINGS = [Binding("escape", "no", "Back")]
    DEFAULT_CSS = """
    ConfirmQuit { align: center middle; }
    #dlg { width: 60; height: auto; border: thick $error; background: $surface; padding: 1 2; }
    """

    def compose(self):
        with Vertical(id="dlg"):
            yield Static("A procedure is running. Abort it (stop + brake) and quit?")
            with Horizontal():
                yield Button("Abort and quit", id="yes", variant="error")
                yield Button("Back", id="no")

    def on_button_pressed(self, ev):
        self.dismiss(ev.button.id == "yes")

    def action_no(self):
        self.dismiss(False)


# ------------------------------------------------------------------------------ app
class KartDashboard(App):
    TITLE = "Kart test bench"
    CSS = """
    #body { height: 1fr; }
    #tabs { width: 1fr; }
    #sidebar { width: 44; border-right: solid $primary-darken-2; padding: 0 1; }
    #proc_list { height: 1fr; min-height: 8; }
    #desc { height: auto; max-height: 12; color: $text-muted; margin: 1 0; }
    #args { margin-bottom: 1; }
    #btnrow Button { width: 1fr; min-width: 8; }
    #btnrow { height: 3; }
    #keys { height: auto; color: $text-muted; margin-top: 1; }
    #state_row { height: 3; }
    #state { width: 1fr; height: 3; padding: 0 1; }
    #dm { width: 26; height: 3; content-align: center middle; text-style: bold; }
    #banner { height: 1; padding: 0 1; }
    #charts { height: 1fr; }
    #angle_col { width: 2fr; }
    #side_col { width: 1fr; }
    #speed { height: 1fr; }
    #track { height: 1fr; }
    #log { height: 14; border-top: solid $primary-darken-2; }
    #health_table { height: 1fr; }
    #cal_table { height: 1fr; }
    #health_btns { height: 3; }
    #health_btns Button { margin-right: 1; }
    #dm_result { height: auto; padding: 0 1; }
    #tree { width: 50; }
    #preview_scroll { width: 1fr; }
    """
    BINDINGS = [
        Binding("escape", "abort", "ABORT"),
        Binding("x", "abort", "Abort", show=False),
        Binding("r", "run", "Run"),
        Binding("h", "health", "Health check"),
        Binding("m", "monitor", "Monitor"),
        Binding("c", "clear", "Clear charts"),
        Binding("q", "quit_app", "Quit"),
    ]

    def __init__(self, cfg_path, sim=False):
        super().__init__()
        self.cfg_path = str(cfg_path)
        self.cfg = load_config(cfg_path)
        self.sim = sim
        self.sim_proc = None
        self.proc = None            # running procedure
        self.proc_name = None
        self.mon = None             # monitor process
        self.monitor_wanted = True
        self.selected = PROCEDURES[0]
        self.args_by_id = {p["id"]: p["args"] for p in PROCEDURES}
        self.tel = {}
        self.status = {}
        self.need_deadman = False
        self.jog_text = ""
        self.last_run_dir = ""
        self.dm_test = None
        self.angle_mode = None

    # ----------------------------------------------------------------- layout
    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id="body"):
            with Vertical(id="sidebar"):
                yield Label(Text("PROCEDURES", style="bold"))
                yield ListView(*[ListItem(Label(self._proc_label(p)), id=f"li_{p['id']}")
                                 for p in PROCEDURES], id="proc_list")
                yield Static("", id="desc")
                yield Label("arguments:")
                yield Input(value=PROCEDURES[0]["args"], id="args")
                with Horizontal(id="btnrow"):
                    yield Button("Run (r)", id="run", variant="success")
                    yield Button("ABORT (esc)", id="abort", variant="error")
                yield Static(Text("SPACE hold = deadman   A/D hold = jog\nENTER = done (jog)   "
                                  "ESC/X = abort\nW/S/B throttle/brake (manual drive)\n"
                                  "M monitor  H health  C clear  Q quit"), id="keys")
            with TabbedContent(id="tabs"):
                with TabPane("Live", id="tab_live"):
                    with Horizontal(id="state_row"):
                        yield Static("", id="state")
                        yield Static("", id="dm")
                    yield Static("", id="banner")
                    with Horizontal(id="charts"):
                        with Vertical(id="angle_col"):
                            yield LineChart("steering", id="angle")
                            yield CommandStrip(id="cmd")
                        with Vertical(id="side_col"):
                            yield LineChart("speed", " km/h", y_min=0.0, id="speed")
                            yield XYChart("track (VectorNav)", id="track")
                    yield RichLog(id="log", wrap=True, markup=False, highlight=False,
                                  max_lines=5000)
                with TabPane("Health", id="tab_health"):
                    with Horizontal(id="health_btns"):
                        yield Button("Run health check", id="hc")
                        yield Button("Health + steering pulse", id="hc_motion")
                        yield Button("Test SPACE auto-repeat", id="dm_btn")
                    yield Static("", id="dm_result")
                    yield DataTable(id="health_table")
                    yield Label(Text("Calibration files (data/calibration)", style="bold"))
                    yield DataTable(id="cal_table")
                with TabPane("Data", id="tab_data"):
                    with Horizontal():
                        yield DirectoryTree(self.cfg["_data_root"], id="tree")
                        with VerticalScroll(id="preview_scroll"):
                            yield Static("select a file", id="preview")
        yield Footer()

    def _proc_label(self, p):
        cal = Calibration(self.cfg)
        missing = [n for n in p["needs"] if not cal.has(n)]
        t = Text(f"{p['id']} ", style="bold")
        t.append(p["title"])
        if p.get("motion"):
            t.append(" [M]", style="yellow")
        if p.get("vn"):
            t.append(" [VN]", style="cyan")
        if missing:
            t.append("  needs " + ",".join(missing), style="dim red")
        return t

    def on_mount(self):
        self.sub_title = f"{self.cfg_path}  ·  port {self.cfg['serial']['port']}"
        a = self.query_one("#angle", LineChart)
        a.add_series("wheel", "bold yellow")
        sp = self.query_one("#speed", LineChart)
        sp.add_series("speed", "bold green")
        sp.hlines = [(15.0, "grey50", "15")]
        ht = self.query_one("#health_table", DataTable)
        ht.add_columns("check", "status", "detail")
        ct = self.query_one("#cal_table", DataTable)
        ct.add_columns("file", "status", "saved", "what")
        self.refresh_cal()
        self._show_desc()
        self.set_interval(0.1, self.refresh_view)
        if self.sim:
            self.run_worker(self.start_sim(), exclusive=False)
        else:
            self.run_worker(self.start_monitor(), exclusive=False)
        self.log_line(f"config {self.cfg_path}; data in {self.cfg['_data_root']}", "==>")
        self.log_line("Start with H (health check). Monitor shows live data while idle.", "==>")

    # --------------------------------------------------------------- helpers
    def log_line(self, text, tag=None):
        style = ""
        for k, s in LEVEL_STYLE.items():
            if (tag or text).startswith(k) or (k == "SAFETY" and "SAFETY" in text):
                style = s
        self.query_one("#log", RichLog).write(Text(text, style=style))

    def _show_desc(self):
        p = self.selected
        t = Text(p["desc"])
        if p["needs"]:
            t.append(f"\nneeds: {', '.join(p['needs'])}", style="dim")
        if p.get("motion"):
            t.append("\n[M] moves the kart (hold SPACE)", style="yellow")
        if p.get("vn"):
            t.append("\n[VN] needs VectorNav", style="cyan")
        self.query_one("#desc", Static).update(t)

    def refresh_cal(self):
        cal = Calibration(self.cfg)
        ct = self.query_one("#cal_table", DataTable)
        ct.clear()
        for name, what in CALIBRATIONS.items():
            d = cal.get(name)
            ct.add_row(name, Text("present", style="green") if d else Text("missing", style="red"),
                       d.get("saved_at", "") if d else "", what)
        for p in PROCEDURES:
            try:
                self.query_one(f"#li_{p['id']}", ListItem).query_one(Label).update(self._proc_label(p))
            except Exception:
                pass
        r = cal.get("range")
        a = self.query_one("#angle", LineChart)
        if r and r.get("lock_left_deg") is not None:
            a.hlines = [(r["lock_left_deg"], "red", "L"), (r["lock_right_deg"], "red", "R")]
        else:
            a.hlines = []

    # ----------------------------------------------------------- processes
    async def start_sim(self):
        run_dir = ROOT / "sim" / "run"
        self.sim_proc = await asyncio.create_subprocess_exec(
            sys.executable, "-u", str(ROOT / "sim" / "fake_kart.py"), "--vn",
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
            start_new_session=True)
        for _ in range(2):
            line = await self.sim_proc.stdout.readline()
            self.log_line("sim: " + line.decode().strip(), "==>")
        await asyncio.sleep(0.3)
        await self.start_monitor()
        _ = run_dir

    async def _spawn(self, script, args):
        cmd = [sys.executable, "-u", str(ROOT / "procedures" / script), "--tui",
               "--config", self.cfg_path, *args]
        return await asyncio.create_subprocess_exec(
            *cmd, stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT, cwd=str(ROOT), limit=8 * 1024 * 1024,
            start_new_session=True)

    async def start_monitor(self):
        if self.mon or self.proc or not self.monitor_wanted:
            return
        if not os.path.exists(self.cfg["serial"]["port"]):
            self.log_line(f"[WARN] monitor: {self.cfg['serial']['port']} not present", "[WARN]")
            return
        self.mon = await self._spawn("monitor.py", [])
        self.run_worker(self.read_proc(self.mon, "monitor"), exclusive=False)

    async def stop_monitor(self):
        m = self.mon
        if not m:
            return
        self.mon = None
        try:
            m.send_signal(signal.SIGINT)
            await asyncio.wait_for(m.wait(), 3)
        except (asyncio.TimeoutError, ProcessLookupError):
            try:
                m.kill()
            except ProcessLookupError:
                pass

    async def start_proc(self, p, extra_args):
        if self.proc:
            self.notify("a procedure is already running", severity="warning")
            return
        await self.stop_monitor()
        try:
            args = shlex.split(extra_args)
        except ValueError as e:
            self.notify(f"bad arguments: {e}", severity="error")
            return
        self.query_one("#angle", LineChart).clear_data()
        self.query_one("#cmd", CommandStrip).clear_data()
        self.query_one("#speed", LineChart).clear_data()
        self.query_one("#track", XYChart).clear_data()
        if p["id"] == "p01":
            self.query_one("#health_table", DataTable).clear()
        self.status, self.need_deadman, self.jog_text = {}, False, ""
        self.proc_name = f"{p['id']} {p['title']}"
        self.log_line(f"==> starting {p['script']} {extra_args}", "==>")
        self.proc = await self._spawn(p["script"], args)
        self.set_focus(None)
        self.query_one("#tabs", TabbedContent).active = "tab_live"
        self.run_worker(self.read_proc(self.proc, "proc"), exclusive=False)

    def send(self, line):
        pr = self.proc
        if pr and pr.stdin and pr.returncode is None:
            try:
                pr.stdin.write((line + "\n").encode())
            except (ConnectionResetError, BrokenPipeError, RuntimeError):
                pass

    async def read_proc(self, pr, kind):
        while True:
            try:
                raw = await pr.stdout.readline()
            except (ValueError, asyncio.LimitOverrunError):
                continue
            if not raw:
                break
            line = raw.decode(errors="replace").rstrip()
            if line.startswith("@"):
                try:
                    self.on_msg(json.loads(line[1:]), kind)
                except json.JSONDecodeError:
                    self.log_line(line)
            elif line:
                self.log_line(line if kind == "proc" else f"monitor: {line}")
        code = await pr.wait()
        if kind == "proc" and pr is self.proc:
            self.proc = None
            self.log_line(f"==> {self.proc_name} exited ({code})", "==>")
            self.proc_name = None
            self.need_deadman, self.jog_text = False, ""
            self.refresh_cal()
            try:
                self.query_one("#tree", DirectoryTree).reload()
            except Exception:
                pass
            await asyncio.sleep(0.3)
            await self.start_monitor()
        elif kind == "monitor" and pr is self.mon:
            self.mon = None

    # -------------------------------------------------------- procedure msgs
    def on_msg(self, m, kind):
        k = m.get("k")
        if k == "tel":
            self.on_tel(m)
        elif k == "prompt" and kind == "proc":
            def done(value, pid=m["id"]):
                self.send(f"answer {pid} {value if value is not None else ''}")
                self.set_focus(None)
            self.push_screen(PromptScreen(m), done)
        elif k == "status":
            self.status.update({kk: v for kk, v in m.items() if kk != "k"})
            if "need_deadman" in m:
                self.need_deadman = m["need_deadman"]
            if "jog" in m:
                self.jog_text = m.get("text", "") if m["jog"] else ""
        elif k == "check":
            st = {"ok": Text("OK", style="bold green"), "warn": Text("WARN", style="bold yellow"),
                  "fail": Text("FAIL", style="bold red")}[m["status"]]
            self.query_one("#health_table", DataTable).add_row(m["name"], st, m["detail"])
        elif k == "result":
            status = m.get("status")
            self.notify(f"{self.proc_name}: {status}",
                        severity="information" if status == "completed" else "error", timeout=8)
            summ = m.get("summary") or {}
            if "overall" in summ:
                self.log_line(f"health: {summ['overall'].upper()}",
                              "[OK]" if summ["overall"] == "ok" else "[WARN]")
            else:
                txt = json.dumps(summ, default=str)
                self.log_line(f"summary: {txt[:1500]}{' ...' if len(txt) > 1500 else ''}")
            if m.get("run_dir"):
                self.last_run_dir = m["run_dir"]
                self.log_line(f"data: {m['run_dir']}", "[OK]")
        elif k == "monitor" and not m.get("ok"):
            self.log_line(f"[WARN] monitor: {m.get('error')}", "[WARN]")

    def on_tel(self, m):
        self.tel = m
        now = time.monotonic()
        if m.get("deg") is not None:
            y, mode = m["deg"], "wheel angle (deg)"
        elif m.get("pos") is not None:
            y, mode = m["pos"], "encoder from zero (ticks)"
        else:
            y, mode = m.get("raw"), "raw encoder (ticks)"
        a = self.query_one("#angle", LineChart)
        if mode != self.angle_mode:
            self.angle_mode = mode
            a.clear_data()
            a.title = f"steering: {mode}"
        a.push("wheel", now, y)
        d = m.get("dir")
        self.query_one("#cmd", CommandStrip).push(now, d, m.get("thr"))
        self.query_one("#speed", LineChart).push("speed", now, m.get("v"))
        self.query_one("#track", XYChart).push(m.get("x"), m.get("y"))

    # --------------------------------------------------------------- display
    def refresh_view(self):
        for wid in ("#angle", "#cmd", "#speed", "#track"):
            self.query_one(wid).refresh()
        t = self.tel
        g = Table.grid(expand=True, padding=(0, 1))
        cols = ["encoder (pos)", "wheel", "command", "thr / brake", "speed", "yaw rate", "links"]
        for c in cols:
            g.add_column(c, ratio=1, no_wrap=True, overflow="ellipsis")
        g.add_row(*[Text(c, style="dim") for c in cols])

        def val(v, fmt="{}", none="-"):
            return none if v is None else fmt.format(v)
        running = self.proc is not None
        dm_need = t.get("need_dm") and running
        if self.need_deadman and running:
            dm, bg, fg = "HOLD SPACE\nto start", "#d7af00", "#000000"
        elif dm_need and t.get("dm"):
            dm, bg, fg = "DEADMAN HELD\nmoving", "#00a000", "#000000"
        elif dm_need:
            dm, bg, fg = "RELEASED\nstopping", "#c00000", "#ffffff"
        elif t.get("jog") and running:
            dm, bg, fg = "JOG\nhold A / D", "#00a0c0", "#000000"
        elif t.get("fault") and running:
            dm, bg, fg = "STOPPED\n" + str(t["fault"])[:22], "#c00000", "#ffffff"
        else:
            dm, bg, fg = "deadman\nnot needed", "#262626", "#808080"
        dmw = self.query_one("#dm", Static)
        dmw.update(Text(dm, justify="center"))
        dmw.styles.background = bg
        dmw.styles.color = fg
        sd = t.get("sd")
        cmd = t.get("dir") or "-"
        cmd_t = Text(f"{cmd} D{sd}" if sd is not None else cmd,
                     style={"left": "bold cyan", "right": "bold magenta"}.get(cmd, ""))
        enc_age = t.get("enc_age")
        links = Text(f"ser {val(t.get('rx_hz'), '{:.0f}')}/s", style="bold red"
                     if enc_age is None or enc_age > 0.5 else "")
        links.append(f"  vn {val(t.get('vn_hz'), '{:.0f}')}/s",
                     style="" if t.get("vn_hz") else "dim")
        if t.get("vn_hz") and not t.get("fix"):
            links.append(" nofix", style="yellow")
        brk = Text(f"{val(t.get('thr'))} / ")
        brk.append("BRAKE" if t.get("brk") else "off", style="bold red" if t.get("brk") else "dim")
        g.add_row(Text(f"{val(t.get('raw'))} ({val(t.get('pos'), '{:+}')})"),
                  Text(val(t.get("deg"), "{:+.2f}°"), style="bold yellow"), cmd_t, brk,
                  Text(val(t.get("v"), "{:.1f} km/h"), style="bold green"),
                  Text(val(t.get("yr"), "{:+.3f} r/s")), links)
        self.query_one("#state", Static).update(g)
        if running:
            phase = self.status.get("phase") or t.get("phase") or ""
            b = Text(f" ▶ {self.proc_name}", style="bold")
            if phase:
                b.append(f"  ·  {phase}")
            if self.jog_text:
                b.append(f"  ·  JOG: {self.jog_text} (hold A/D, ENTER when done)",
                         style="bold cyan")
            elif self.need_deadman:
                b.append("  ·  hold SPACE to start", style="bold yellow")
            if t.get("fault"):
                b.append(f"  ·  {t['fault']}", style="bold red")
        elif self.mon:
            b = Text(" monitor: live data, nothing sent", style="dim")
            line = t.get("line")
            if line:
                b.append(f"   last line: {line}", style="dim")
        else:
            b = Text(" idle (monitor off: M)", style="dim")
        self.query_one("#banner", Static).update(b)

    # ------------------------------------------------------------------ input
    def on_key(self, ev: events.Key):
        if isinstance(self.focused, Input) or isinstance(self.screen, ModalScreen):
            return
        k = ev.key
        if self.dm_test is not None and k == "space":
            self.dm_test.append(time.monotonic())
            ev.stop()
            return
        if not self.proc:
            return
        if k == "space":
            self.send("hb")
        elif k in ("a", "left"):
            self.send("jog left")
        elif k in ("d", "right"):
            self.send("jog right")
        elif k == "enter":
            self.send("key enter")
        elif k in ("w", "s", "b"):
            self.send(f"key {k}")
        else:
            return
        ev.stop()

    def on_list_view_highlighted(self, ev: ListView.Highlighted):
        if ev.item is None:
            return
        pid = ev.item.id.removeprefix("li_")
        inp = self.query_one("#args", Input)
        self.args_by_id[self.selected["id"]] = inp.value
        self.selected = next(p for p in PROCEDURES if p["id"] == pid)
        inp.value = self.args_by_id[pid]
        self._show_desc()

    def on_input_submitted(self, ev: Input.Submitted):
        if ev.input.id == "args":
            self.action_run()

    def on_button_pressed(self, ev: Button.Pressed):
        bid = ev.button.id
        if bid == "run":
            self.action_run()
        elif bid == "abort":
            self.action_abort()
        elif bid in ("hc", "hc_motion"):
            p = next(p for p in PROCEDURES if p["id"] == "p01")
            self.run_worker(self.start_proc(p, "--motion-check" if bid == "hc_motion" else ""))
        elif bid == "dm_btn":
            self.run_worker(self.deadman_test())
        self.set_focus(None)

    def on_directory_tree_file_selected(self, ev: DirectoryTree.FileSelected):
        p = Path(ev.path)
        try:
            if p.suffix == ".json":
                txt = json.dumps(json.loads(p.read_text()), indent=2)[:20000]
            elif p.suffix == ".csv":
                lines = p.read_text(errors="replace").splitlines()
                txt = f"{len(lines) - 1} rows\n" + "\n".join(lines[:60])
            else:
                txt = p.read_text(errors="replace")[:20000]
        except Exception as e:
            txt = f"cannot read: {e}"
        self.query_one("#preview", Static).update(Text(f"{p}\n\n{txt}"))

    async def deadman_test(self):
        if self.proc:
            self.notify("not while a procedure runs", severity="warning")
            return
        res = self.query_one("#dm_result", Static)
        res.update(Text("Hold SPACE now for 3 seconds...", style="bold yellow"))
        self.set_focus(None)
        self.dm_test = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < 8:
            await asyncio.sleep(0.05)
            if self.dm_test and time.monotonic() - self.dm_test[0] > 3.2:
                break
        ts, self.dm_test = self.dm_test, None
        sc = self.cfg["safety"]
        if len(ts) < 5:
            res.update(Text(f"Only {len(ts)} SPACE events: auto-repeat looks OFF. The deadman will "
                            "not work: enable key repeat in your OS / terminal.", style="bold red"))
            return
        first = ts[1] - ts[0]
        gaps = [b - a for a, b in zip(ts[1:], ts[2:])]
        gaps.sort()
        med, worst = gaps[len(gaps) // 2], gaps[int(len(gaps) * 0.95)]
        ok = first < sc["deadman_initial_s"] and worst < sc["deadman_repeat_s"]
        res.update(Text(
            f"first repeat after {first * 1000:.0f} ms (limit {sc['deadman_initial_s'] * 1000:.0f}), "
            f"repeat every {med * 1000:.0f} ms, worst {worst * 1000:.0f} ms (limit "
            f"{sc['deadman_repeat_s'] * 1000:.0f}). " + ("OK" if ok else
                                                         "Too slow: raise the limits in config.toml "
                                                         "[safety] or speed up key repeat."),
            style="bold green" if ok else "bold red"))

    # ---------------------------------------------------------------- actions
    def action_run(self):
        if self.proc:
            self.notify("a procedure is running", severity="warning")
            return
        inp = self.query_one("#args", Input)
        self.args_by_id[self.selected["id"]] = inp.value
        self.run_worker(self.start_proc(self.selected, inp.value))

    def action_abort(self):
        if not self.proc:
            return
        self.send("abort")
        self.log_line("[ERROR] ABORT sent", "[ERROR]")
        pr = self.proc

        async def escalate():
            await asyncio.sleep(4)
            if pr.returncode is None:
                pr.send_signal(signal.SIGTERM)
                self.log_line("[ERROR] procedure did not stop: SIGTERM", "[ERROR]")
        self.run_worker(escalate())

    def action_health(self):
        if self.proc:
            return
        p = next(p for p in PROCEDURES if p["id"] == "p01")
        self.run_worker(self.start_proc(p, ""))

    def action_monitor(self):
        if self.proc:
            return
        self.monitor_wanted = not self.monitor_wanted
        if self.monitor_wanted:
            self.run_worker(self.start_monitor())
        else:
            self.run_worker(self.stop_monitor())

    def action_clear(self):
        for wid in ("#angle", "#cmd", "#speed", "#track"):
            self.query_one(wid).clear_data()

    def action_quit_app(self):
        if self.proc:
            def done(yes):
                if yes:
                    self.run_worker(self.shutdown())
            self.push_screen(ConfirmQuit(), done)
        else:
            self.run_worker(self.shutdown())

    async def shutdown(self):
        if self.proc:
            self.send("abort")
            try:
                await asyncio.wait_for(self.proc.wait(), 6)
            except asyncio.TimeoutError:
                self.proc.send_signal(signal.SIGTERM)
        await self.stop_monitor()
        if self.sim_proc and self.sim_proc.returncode is None:
            self.sim_proc.terminate()
        self.exit()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--sim", action="store_true", help="start the fake kart and use sim config")
    a = ap.parse_args()
    cfg = a.config or (ROOT / "sim" / "sim_config.toml" if a.sim else ROOT / "config.toml")
    KartDashboard(cfg, sim=a.sim).run()


if __name__ == "__main__":
    main()
