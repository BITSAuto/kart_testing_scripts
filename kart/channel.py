"""Talking to the operator: the dashboard (--tui) or a plain terminal.

Dashboard mode, procedure -> dashboard (stdout):
    plain text lines              shown in the log pane
    @{"k": "tel", ...}            live telemetry (20 Hz)
    @{"k": "prompt", ...}         a question; the dashboard opens a dialog
    @{"k": "status", ...}         phase / trial counter / deadman request
    @{"k": "check", ...}          a health check result
    @{"k": "result", ...}         final summary

Dashboard -> procedure (stdin), one per line:
    hb                    SPACE is held (sent on every key repeat)
    jog left|right|stop   jog keys (sent on every key repeat)
    key <name>            other keys (manual drive)
    answer <id> <text>    reply to a prompt
    abort                 stop now and end the procedure
End of stdin (dashboard gone) counts as abort.
"""
import json
import queue
import sys
import threading
import time
import uuid


class Aborted(Exception):
    """End the procedure (operator abort, fault)."""


class Paused(Exception):
    """Motion stopped because the deadman was released; the trial can be redone."""


class Channel:
    def __init__(self, tui: bool, cfg: dict):
        self.tui = tui
        sc = cfg["safety"]
        self.dm_initial, self.dm_repeat = sc["deadman_initial_s"], sc["deadman_repeat_s"]
        self.abort_evt = threading.Event()
        self.abort_reason = ""
        self.hb_t = 0.0             # last SPACE event
        self.hb_streak = 0          # SPACE events in the current hold
        self.jog_dir, self.jog_t, self.jog_streak = "stop", 0.0, 0
        self.keys = queue.Queue()
        self.last_input_t = 0.0
        self._answers = queue.Queue()
        self._out = threading.Lock()
        if tui:
            threading.Thread(target=self._stdin, daemon=True).start()

    # ------------------------------------------------------------------- input
    def _stdin(self):
        for line in sys.stdin:
            parts = line.strip().split(" ", 2)
            if not parts or not parts[0]:
                continue
            now = time.monotonic()
            self.last_input_t = now
            cmd = parts[0]
            if cmd == "hb":
                gap = now - self.hb_t
                self.hb_streak = self.hb_streak + 1 if gap < max(self.dm_initial, self.dm_repeat) else 1
                self.hb_t = now
            elif cmd == "jog" and len(parts) > 1:
                same = parts[1] == self.jog_dir and now - self.jog_t < self.dm_initial
                self.jog_streak = self.jog_streak + 1 if same else 1
                self.jog_dir, self.jog_t = parts[1], now
            elif cmd == "key" and len(parts) > 1:
                self.keys.put((now, parts[1]))
            elif cmd == "answer" and len(parts) > 1:
                self._answers.put((parts[1], parts[2] if len(parts) > 2 else ""))
            elif cmd == "abort":
                self.abort("operator abort")
        self.abort("dashboard closed (stdin ended)")

    def abort(self, reason):
        if not self.abort_evt.is_set():
            self.abort_reason = reason
            self.abort_evt.set()

    def deadman_held(self, enabled=True) -> bool:
        if not enabled:
            return True
        dt = time.monotonic() - self.hb_t
        return dt < (self.dm_initial if self.hb_streak <= 1 else self.dm_repeat)

    def jog_state(self, release_s, initial_s):
        """Direction the jog keys ask for right now ('left', 'right' or 'stop')."""
        if self.jog_dir == "stop":
            return "stop"
        dt = time.monotonic() - self.jog_t
        return self.jog_dir if dt < (initial_s if self.jog_streak <= 1 else release_s) else "stop"

    # ------------------------------------------------------------------ output
    def emit(self, _kind, **data):
        if self.tui:
            with self._out:
                try:
                    sys.stdout.write("@" + json.dumps({"k": _kind, **data}, default=str) + "\n")
                    sys.stdout.flush()
                except (BrokenPipeError, OSError, ValueError):
                    pass

    def log(self, msg, level="info"):
        prefix = {"info": "", "warn": "[WARN] ", "error": "[ERROR] ", "ok": "[OK] ",
                  "step": "==> "}.get(level, "")
        with self._out:
            try:
                sys.stdout.write(prefix + str(msg) + "\n")
                sys.stdout.flush()
            except (BrokenPipeError, OSError, ValueError):
                pass

    def status(self, **data):
        self.emit("status", **data)

    # ----------------------------------------------------------------- prompts
    def ask(self, text, kind="text", choices=None, default=None, allow_empty=False):
        """Ask the operator. kind: text | number | confirm | choice. Blocks until answered."""
        if self.tui:
            pid = uuid.uuid4().hex[:8]
            self.emit("prompt", id=pid, text=text, kind=kind, choices=choices, default=default)
            while True:
                if self.abort_evt.is_set():
                    raise Aborted(self.abort_reason)
                try:
                    aid, value = self._answers.get(timeout=0.1)
                except queue.Empty:
                    continue
                if aid == pid:
                    break
        else:
            hint = {"confirm": " [y/n]", "number": " (number)"}.get(kind, "")
            if choices:
                hint = " [" + "/".join(choices) + "]"
            if default is not None:
                hint += f" (default {default})"
            try:
                value = input(f"\n>>> {text}{hint}: ").strip()
            except EOFError:
                raise Aborted("stdin closed")
        if value == "" and default is not None:
            value = str(default)
        if value == "__abort__":
            self.abort("operator abort")
            raise Aborted("operator abort")
        if kind == "number":
            try:
                return float(value)
            except ValueError:
                if allow_empty and value == "":
                    return None
                self.log(f"'{value}' is not a number, try again", "warn")
                return self.ask(text, kind, choices, default, allow_empty)
        if kind == "confirm":
            return value.lower() in ("y", "yes", "1", "true", "ok")
        if kind == "choice" and choices and value not in choices:
            self.log(f"choose one of {choices}", "warn")
            return self.ask(text, kind, choices, default, allow_empty)
        return value

    def confirm(self, text, default=None):
        return self.ask(text, "confirm", default=default)
