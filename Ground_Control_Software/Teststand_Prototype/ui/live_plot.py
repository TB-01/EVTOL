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

        self.fig = Figure(figsize=(5, 3), dpi=120)
        self.fig.set_constrained_layout(False)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title(title)
        self.ax.set_xlabel("time [s]", fontsize=13)
        self.ax.set_ylabel(y_label, fontsize=13)          # <-- use provided units
        self.ax.tick_params(axis='both', labelsize=12)
        (self.line,) = self.ax.plot([], [], lw=1.6)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # --- Throttle overlay (right y-axis) ---
        self.ax2 = self.ax.twinx()
        self.ax2.set_ylabel("% throttle", fontsize=13)
        self.ax2.tick_params(axis='y', labelsize=12)
        self.ax2.set_ylim(0, 100)

        # separate buffers for throttle overlay
        self.buf_th1 = deque()
        self.buf_th2 = deque()

        # 2 overlay lines (styles only; default colors)
        (self.line_th1,) = self.ax2.plot([], [], lw=1.0, linestyle="--")
        (self.line_th2,) = self.ax2.plot([], [], lw=1.0, linestyle=":")

        self._min_dt = 1.0 / max(5, fps)
        self._last_draw = 0.0
        self._resizing_until = 0.0
        self.fig.canvas.mpl_connect("resize_event", self._on_mpl_resize)
        self.after(75, self._tick)

        self.mode = "live"        # "live" (rolling window) or "fixed"
        self.fixed_total_s = None # used when mode == "fixed"

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
        self.buf_th1.clear()
        self.buf_th2.clear()
        self.line.set_data([], [])
        self.line_th1.set_data([], [])
        self.line_th2.set_data([], [])
        self.ax.relim(); self.ax.autoscale_view()
        self.ax2.set_ylim(0, 100)
        self.canvas.draw_idle()

    def add_sample(self, host_ms: int, y):
        if self.t0 is None:
            self.t0 = host_ms
        t = (host_ms - self.t0) / 1000.0
        self.buf.append((t, y))
        if self.mode == "live":
            # rolling window
            tmin = t - self.window_s
            while self.buf and self.buf[0][0] < tmin:
                self.buf.popleft()
        else:
            # fixed mode: keep all; no pop
            pass

    def _render_overlay(self, buf):
        n = len(buf)
        if n == 0:
            return [], []
        stride = max(1, math.ceil(n / self.max_points))
        xs = [buf[i][0] for i in range(0, n, stride)]
        ys = [buf[i][1] for i in range(0, n, stride)]
        return xs, ys

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
            self.after(75, self._tick); return
        if (now - self._last_draw) < self._min_dt:
            self.after(25, self._tick); return

        xs, ys   = self._render_data()
        xs1, ys1 = self._render_overlay(self.buf_th1)
        xs2, ys2 = self._render_overlay(self.buf_th2)

        # update all three lines regardless of which has data
        self.line.set_data(xs, ys)
        self.line_th1.set_data(xs1, ys1)
        self.line_th2.set_data(xs2, ys2)

        # choose the most recent x from any buffer
        any_xs = xs or xs1 or xs2
        if any_xs:
            if self.mode == "fixed" and self.fixed_total_s:
                left, right = 0, self.fixed_total_s
            else:
                xlast = (xs[-1] if xs else (xs1[-1] if xs1 else xs2[-1]))
                left  = max(0, xlast - self.window_s)
                right = max(self.window_s, xlast)
            if self.ax.get_xlim() != (left, right):
                self.ax.set_xlim(left, right)

        # y-limits only from thrust curve if present (keeps overlay uncluttered)
        if ys:
            ymin, ymax = min(ys), max(ys)
            if ymin == ymax:
                ymin -= 1; ymax += 1
            pad = 0.05 * (ymax - ymin)
            new_ylim = (ymin - pad, ymax + pad)
            if self.ax.get_ylim() != new_ylim:
                self.ax.set_ylim(*new_ylim)

        # keep right axis fixed
        self.ax2.set_ylim(0, 100)

        self.canvas.draw_idle()
        self._last_draw = now
        self.after(75, self._tick)

    
    def set_mode_live(self, window_s=None):
        self.mode = "live"
        if window_s is not None:
            self.window_s = float(window_s)
        self.fixed_total_s = None
        self.clear()

    def set_mode_fixed(self, total_s: float):
        """Fixed x-axis [0..total_s]."""
        self.mode = "fixed"
        self.fixed_total_s = float(total_s)
        self.t0 = None

        # clear all buffers (main + overlay)
        self.buf.clear()
        self.buf_th1.clear()
        self.buf_th2.clear()

        self.line.set_data([], [])
        self.line_th1.set_data([], [])
        self.line_th2.set_data([], [])

        self.ax.set_xlim(0, max(1.0, self.fixed_total_s))
        self.ax2.set_ylim(0, 100)
        self.canvas.draw_idle()


    def add_throttle(self, host_ms: int, m1_pct=None, m2_pct=None):
        """Overlay throttle % for M1/M2 at the same x (time) as the thrust data."""
        if self.t0 is None:
            self.t0 = host_ms
        t = (host_ms - self.t0) / 1000.0
        if m1_pct is not None:
            self.buf_th1.append((t, float(m1_pct)))
        if m2_pct is not None:
            self.buf_th2.append((t, float(m2_pct)))

        if self.mode == "live":
            tmin = t - self.window_s
            while self.buf_th1 and self.buf_th1[0][0] < tmin:
                self.buf_th1.popleft()
            while self.buf_th2 and self.buf_th2[0][0] < tmin:
                self.buf_th2.popleft()
        # fixed mode: keep all
