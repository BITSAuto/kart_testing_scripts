"""Terminal charts drawn with braille characters (2x4 dots per cell). No GPU, no extra deps."""
from collections import deque

from rich.text import Text
from textual.widget import Widget

DOTS = ((0x01, 0x08), (0x02, 0x10), (0x04, 0x20), (0x40, 0x80))   # [row][col]


class Canvas:
    def __init__(self, cols, rows):
        self.cols, self.rows = max(cols, 1), max(rows, 1)
        self.w, self.h = self.cols * 2, self.rows * 4
        self.bits = [[0] * self.cols for _ in range(self.rows)]
        self.color = [[None] * self.cols for _ in range(self.rows)]

    def dot(self, x, y, color):
        x, y = int(round(x)), int(round(y))
        if 0 <= x < self.w and 0 <= y < self.h:
            r, c = y // 4, x // 2
            self.bits[r][c] |= DOTS[y % 4][x % 2]
            self.color[r][c] = color

    def line(self, x0, y0, x1, y1, color):
        x0, y0, x1, y1 = (int(round(v)) for v in (x0, y0, x1, y1))
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
        err = dx + dy
        for _ in range(4 * (self.w + self.h)):
            self.dot(x0, y0, color)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def hline(self, y, color, dash=3):
        for x in range(0, self.w, dash):
            self.dot(x, y, color)

    def rows_text(self):
        out = []
        for r in range(self.rows):
            t = Text()
            for c in range(self.cols):
                b = self.bits[r][c]
                t.append(chr(0x2800 + b) if b else " ", style=self.color[r][c] or "")
            out.append(t)
        return out


def _fmt(v):
    if v is None:
        return "-"
    a = abs(v)
    return f"{v:.0f}" if a >= 100 else (f"{v:.1f}" if a >= 10 else f"{v:.2f}")


class LineChart(Widget):
    """Scrolling time chart. push(name, t, y) with t in seconds (any monotonic clock)."""
    DEFAULT_CSS = "LineChart { height: 1fr; width: 1fr; min-height: 6; }"

    def __init__(self, title, unit="", window=30.0, y_min=None, y_max=None, **kw):
        super().__init__(**kw)
        self.title, self.unit, self.window = title, unit, window
        self.y_min, self.y_max = y_min, y_max
        self.series = {}
        self.hlines = []        # (y, style, label)
        self.note = ""

    def add_series(self, name, style):
        self.series[name] = (style, deque(maxlen=6000))

    def push(self, name, t, y):
        self.series[name][1].append((t, y))

    def clear_data(self):
        for _, d in self.series.values():
            d.clear()

    def render(self):
        W, H = self.size.width, self.size.height
        left = 8
        cols, rows = max(W - left, 4), max(H - 2, 2)
        t_now = max((d[-1][0] for _, d in self.series.values() if d), default=0.0)
        t0 = t_now - self.window
        vis = {n: [(t, y) for t, y in d if t >= t0 and y is not None]
               for n, (_, d) in self.series.items()}
        ys = [y for pts in vis.values() for _, y in pts] + [h[0] for h in self.hlines]
        lo = self.y_min if self.y_min is not None else (min(ys) if ys else 0.0)
        hi = self.y_max if self.y_max is not None else (max(ys) if ys else 1.0)
        if self.y_min is None and ys:
            lo = min(lo, min(ys))
        if self.y_max is None and ys:
            hi = max(hi, max(ys))
        if hi - lo < 1e-9:
            lo, hi = lo - 1, hi + 1
        pad = (hi - lo) * 0.08
        if self.y_min is None:
            lo -= pad
        if self.y_max is None:
            hi += pad
        cv = Canvas(cols, rows)

        def px(t):
            return (t - t0) / self.window * (cv.w - 1)

        def py(y):
            return (hi - y) / (hi - lo) * (cv.h - 1)
        if lo < 0 < hi:
            cv.hline(py(0), "grey37", 2)
        for y, style, _ in self.hlines:
            if lo <= y <= hi:
                cv.hline(py(y), style, 4)
        for n, pts in vis.items():
            style = self.series[n][0]
            prev = None
            for t, y in pts:
                p = (px(t), py(y))
                if prev and p[0] - prev[0] < cv.w * 0.2:
                    cv.line(prev[0], prev[1], p[0], p[1], style)
                else:
                    cv.dot(p[0], p[1], style)
                prev = p
        head = Text(f" {self.title}", style="bold")
        for n, (style, d) in self.series.items():
            last = next((y for _, y in reversed(d) if y is not None), None)
            head.append(f"  ● {n} ", style=style)
            head.append(f"{_fmt(last)}{self.unit}", style="bold " + style)
        if self.note:
            head.append(f"   {self.note}", style="dim")
        lines = [head]
        body = cv.rows_text()
        for i, row in enumerate(body):
            if i == 0:
                lab = _fmt(hi)
            elif i == len(body) - 1:
                lab = _fmt(lo)
            elif i == len(body) // 2:
                lab = _fmt((hi + lo) / 2)
            else:
                lab = ""
            t = Text(f"{lab:>{left - 1}}│", style="grey50")
            t.append_text(row)
            lines.append(t)
        axis = Text(" " * (left - 1) + "└" + f"-{self.window:.0f}s".ljust(cols - 3) + "now",
                    style="grey50")
        lines.append(axis)
        return Text("\n").join(lines[:H])


class CommandStrip(Widget):
    """One row: which steering command was active over the chart window."""
    DEFAULT_CSS = "CommandStrip { height: 1; width: 1fr; }"
    GLYPH = {"left": ("◀", "bold cyan"), "right": ("▶", "bold magenta"), "stop": ("·", "grey50")}

    def __init__(self, window=30.0, **kw):
        super().__init__(**kw)
        self.window = window
        self.data = deque(maxlen=6000)     # (t, dir, throttle)

    def push(self, t, direction, thr):
        self.data.append((t, direction, thr))

    def clear_data(self):
        self.data.clear()

    def render(self):
        W = self.size.width
        left = 8
        cols = max(W - left, 1)
        out = Text(f"{'cmd':>{left - 1}}│", style="grey50")
        if not self.data:
            return out
        t_now = self.data[-1][0]
        t0 = t_now - self.window
        pts = [d for d in self.data if d[0] >= t0 - 1]
        j = 0
        cur = pts[0] if pts else None
        for c in range(cols):
            tc = t0 + (c + 0.5) / cols * self.window
            while j < len(pts) and pts[j][0] <= tc:
                cur = pts[j]
                j += 1
            if cur is None or cur[0] > tc:
                out.append(" ")
                continue
            d = cur[1] if cur[1] in self.GLYPH else ("stop" if cur[1] in ("stop", "-", None)
                                                      else "left" if "left" in str(cur[1]) else "stop")
            g, style = self.GLYPH.get(d, ("?", "yellow"))
            if cur[2]:
                style += " on grey19"
            out.append(g, style=style)
        return out


class XYChart(Widget):
    """Top-down track (x east, y north), equal scale."""
    DEFAULT_CSS = "XYChart { height: 1fr; width: 1fr; min-height: 6; }"

    def __init__(self, title="track", keep=4000, **kw):
        super().__init__(**kw)
        self.title = title
        self.pts = deque(maxlen=keep)

    def push(self, x, y):
        if x is not None and y is not None:
            self.pts.append((x, y))

    def clear_data(self):
        self.pts.clear()

    def render(self):
        W, H = self.size.width, self.size.height
        cv = Canvas(max(W, 2), max(H - 1, 2))
        head = Text(f" {self.title}", style="bold")
        if not self.pts:
            head.append("  (no position data)", style="dim")
            return head
        xs, ys = [p[0] for p in self.pts], [p[1] for p in self.pts]
        cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        span = max(max(xs) - min(xs), max(ys) - min(ys), 4.0) * 1.15
        scale = min(cv.w, cv.h) / span
        prev = None
        for x, y in self.pts:
            p = (cv.w / 2 + (x - cx) * scale, cv.h / 2 - (y - cy) * scale)
            if prev:
                cv.line(prev[0], prev[1], p[0], p[1], "green")
            prev = p
        cv.dot(prev[0], prev[1], "bold yellow")
        cv.dot(prev[0] + 1, prev[1], "bold yellow")
        head.append(f"  {span:.0f} m across", style="dim")
        return Text("\n").join([head] + cv.rows_text())
