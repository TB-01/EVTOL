# ui/live_plot.py
import math, time
from collections import deque
import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

class LivePlotFrame(ctk.CTkFrame):
    def __init__(self, master, window_s=5.0, max_points=600,
                 title="Thrust (kg)", y_label="kg", fps=25):
        super().__init__(master)
        self.window_s = window_s
        self.max_points = max_points
        self.t0 = None
        self.buf = deque()

        self.fig = Figure(figsize=(5, 3), dpi=90)
        self.fig.set_constrained_layout(False)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title(title)
        self.ax.set_xlabel("time [s]")
        self.ax.set_ylabel(y_label)          # <-- use provided units
        (self.line,) = self.ax.plot([], [], lw=1.2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self._min_dt = 1.0 / max(5, fps)
        self._last_draw = 0.0
        self._resizing_until = 0.0
        self.fig.canvas.mpl_connect("resize_event", self._on_mpl_resize)
        self.after(75, self._tick)

    def set_units(self, title=None, y_label=None):
        """Change title / y-axis label on the fly (e.g., after calibration)."""
        if title:
            self.ax.set_title(title)
        if y_label:
            self.ax.set_ylabel(y_label)
        self.canvas.draw_idle()

    def _on_mpl_resize(self, _evt):
        # pause drawing while a resize storm is happening
        self._resizing_until = time.time() + 0.20

    def clear(self):
        self.t0 = None
        self.buf.clear()
        self.line.set_data([], [])
        self.ax.relim(); self.ax.autoscale_view()
        self.canvas.draw_idle()

    def add_sample(self, host_ms: int, y):
        if self.t0 is None:
            self.t0 = host_ms
        t = (host_ms - self.t0) / 1000.0
        self.buf.append((t, y))
        # rolling window
        tmin = t - self.window_s
        while self.buf and self.buf[0][0] < tmin:
            self.buf.popleft()

    def _render_data(self):
        n = len(self.buf)
        if n == 0:
            return [], []
        stride = max(1, math.ceil(n / self.max_points))
        xs = [self.buf[i][0] for i in range(0, n, stride)]
        ys = [self.buf[i][1] for i in range(0, n, stride)]
        return xs, ys

    def _tick(self):
        now = time.time()
        if now < self._resizing_until:
            # skip drawing during active resize
            self.after(75, self._tick)
            return
        if (now - self._last_draw) < self._min_dt:
            self.after(25, self._tick)
            return

        xs, ys = self._render_data()
        if xs:
            self.line.set_data(xs, ys)

            # x-limits: rolling window
            left = max(0, xs[-1] - self.window_s)
            right = max(self.window_s, xs[-1])
            # only adjust if changed significantly (avoids heavy relayout)
            if self.ax.get_xlim() != (left, right):
                self.ax.set_xlim(left, right)

            # y-limits pad
            ymin, ymax = min(ys), max(ys)
            if ymin == ymax:
                ymin -= 1; ymax += 1
            pad = 0.05 * (ymax - ymin)
            cur_ylim = self.ax.get_ylim()
            new_ylim = (ymin - pad, ymax + pad)
            if (abs(cur_ylim[0]-new_ylim[0]) > 1e-6) or (abs(cur_ylim[1]-new_ylim[1]) > 1e-6):
                self.ax.set_ylim(*new_ylim)

            self.canvas.draw_idle()
            self._last_draw = now

        self.after(75, self._tick)
