# app.py — UI layout per your sketch (CustomTkinter)
import time, queue
import customtkinter as ctk
from core.link import Link
from ui.terminal import TerminalFrame
from ui.live_plot import LivePlotFrame

import os, json, time, queue
# (you already import time, queue — keeping them here for clarity)

CAL_DIR  = os.path.join(os.path.expanduser("~"), ".teststand")
CAL_FILE = os.path.join(CAL_DIR, "calibration.json")
CAL_VERSION = 1

MAX_LOG_LINES = 800   # keep the log snappy

def label_row(parent, title, unit=None, var=None):
    """One-line 'title  [value]  unit' row."""
    frame = ctk.CTkFrame(parent, fg_color="transparent")
    frame.pack(fill="x", padx=8, pady=2)
    ctk.CTkLabel(frame, text=title, width=120, anchor="w").pack(side="left")
    val_lbl = ctk.CTkLabel(frame, textvariable=var, text="—", width=90, anchor="e")
    val_lbl.pack(side="left", padx=(6,6))
    if unit:
        ctk.CTkLabel(frame, text=unit, width=30, anchor="w").pack(side="left")
    return val_lbl

class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Teststand Control")
        self.geometry("1200x720")
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        # calibration state
        self.tare_offset = None           # counts
        self.kg_per_count = None          # scale (kg / count)
        self._cap_mode = None             # "tare" | "cal" | None
        self._cap_samples = []            # collected load_raw
        self._cap_deadline_ms = 0
        self._cap_min_samples = 0

        # thread → UI queue
        self.q = queue.Queue()
        self.link = Link(on_event=self._enqueue)
        self._reqid = 0



        # ====== MAIN GRID: 2 columns (left content, right telemetry) + bottom control row ======
        root = ctk.CTkFrame(self)
        root.pack(fill="both", expand=True, padx=10, pady=10)

        root.grid_columnconfigure(0, weight=1)
        root.grid_rowconfigure(0, weight=1)

        # LEFT: tabs (live plot, terminal)
        left = ctk.CTkFrame(root)
        left.grid(row=0, column=0, sticky="nsew")

        tabs = ctk.CTkTabview(left)
        tabs.pack(fill="both", expand=True, padx=6, pady=6)
        tab_plot = tabs.add("Live Plot")
        tab_term = tabs.add("Terminal")

        self.plot = LivePlotFrame(tab_plot, window_s=5.0, max_points=600, title="Load (raw)")
        self.plot.pack(fill="both", expand=True, padx=6, pady=6)

        self.log = ctk.CTkTextbox(tab_term, wrap="none")
        self.log.pack(side="top", fill="both", expand=True, padx=6, pady=(6,0))
        self.term = TerminalFrame(tab_term, on_command=self._handle_command)
        self.term.pack(side="top", fill="x", padx=6, pady=6)

        # RIGHT: telemetry column
        right = ctk.CTkFrame(root, width=320)
        right.grid(row=0, column=1, sticky="ns", padx=(10,0))
        ctk.CTkLabel(right, text="Motor 1", font=ctk.CTkFont(size=18, weight="bold")).pack(anchor="w", padx=8, pady=(8,0))
        box_m1 = ctk.CTkFrame(right)
        box_m1.pack(fill="x", padx=6, pady=6)

        self.m1_throttle = ctk.StringVar(value="—")
        self.m1_speed    = ctk.StringVar(value="—")
        self.m1_voltage  = ctk.StringVar(value="—")
        self.m1_current  = ctk.StringVar(value="—")
        self.m1_power    = ctk.StringVar(value="—")
        label_row(box_m1, "Throttle", "%", self.m1_throttle)
        label_row(box_m1, "Motor Speed", "RPM", self.m1_speed)
        label_row(box_m1, "Voltage", "V", self.m1_voltage)
        label_row(box_m1, "Current", "A", self.m1_current)
        label_row(box_m1, "Power", "W", self.m1_power)

        ctk.CTkLabel(right, text="Motor 2", font=ctk.CTkFont(size=18, weight="bold")).pack(anchor="w", padx=8, pady=(8,0))
        box_m2 = ctk.CTkFrame(right)
        box_m2.pack(fill="x", padx=6, pady=6)

        self.m2_throttle = ctk.StringVar(value="—")
        self.m2_speed    = ctk.StringVar(value="—")
        self.m2_voltage  = ctk.StringVar(value="—")
        self.m2_current  = ctk.StringVar(value="—")
        self.m2_power    = ctk.StringVar(value="—")
        label_row(box_m2, "Throttle", "%", self.m2_throttle)
        label_row(box_m2, "Motor Speed", "RPM", self.m2_speed)
        label_row(box_m2, "Voltage", "V", self.m2_voltage)
        label_row(box_m2, "Current", "A", self.m2_current)
        label_row(box_m2, "Power", "W", self.m2_power)

        ctk.CTkLabel(right, text="Totals", font=ctk.CTkFont(size=18, weight="bold")).pack(anchor="w", padx=8, pady=(8,0))
        box_tot = ctk.CTkFrame(right)
        box_tot.pack(fill="x", padx=6, pady=6)
        self.total_thrust = ctk.StringVar(value="—")
        self.thrust_per_w = ctk.StringVar(value="—")
        self.total_power  = ctk.StringVar(value="—")
        label_row(box_tot, "Total Thrust", "kg", self.total_thrust)
        label_row(box_tot, "Thrust/Power", "g/W", self.thrust_per_w)
        label_row(box_tot, "Total Power", "W", self.total_power)

        # (Optional) keep a raw readout while sensors are not mapped yet
        self.load_raw_var = ctk.StringVar(value="—")
        label_row(box_tot, "Load raw", None, self.load_raw_var)

        # ==== BOTTOM CONTROLS: left (tests/cal), right (COM) ====
        bottom = ctk.CTkFrame(root)
        bottom.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(10,0))
        bottom.grid_columnconfigure(0, weight=1)
        bottom.grid_columnconfigure(1, weight=0)

        bl = ctk.CTkFrame(bottom)  # bottom-left
        bl.grid(row=0, column=0, sticky="ew", padx=(0,8))
        bl.grid_columnconfigure(6, weight=1)

        # Manual throttle
        ctk.CTkLabel(bl, text="Manual throttle").grid(row=0, column=0, sticky="w", padx=8, pady=(8,2))
        self.manual_thr = ctk.CTkEntry(bl, width=70)
        self.manual_thr.insert(0, "80")
        self.manual_thr.grid(row=1, column=0, sticky="w", padx=8)
        ctk.CTkLabel(bl, text="%").grid(row=1, column=1, sticky="w")

        self.btn_start = ctk.CTkButton(bl, text="Start Test", command=self._not_implemented)
        self.btn_start.grid(row=3, column=2, padx=10)

        self.btn_tare = ctk.CTkButton(bl, text="Tare", command=self._on_tare)
        self.btn_tare.grid(row=1, column=4, padx=10)


        

        # Test mode + Save
        ctk.CTkLabel(bl, text="Test mode").grid(row=2, column=0, sticky="w", padx=8, pady=(8,2))
        self.test_mode = ctk.CTkOptionMenu(bl, values=["Manual","Throttle Range","Static Test","Dynamic Response"])
        self.test_mode.set("Manual")
        self.test_mode.grid(row=3, column=0, sticky="w", padx=8)
        self.btn_save = ctk.CTkButton(bl, text="Save Result", command=self._not_implemented)
        self.btn_save.grid(row=3, column=3, padx=10)

        # Calibration
        ctk.CTkLabel(bl, text="Calibration Weight").grid(row=2, column=4, sticky="w", padx=8, pady=(8,2))
        self.cal_weight = ctk.CTkEntry(bl, width=70)
        self.cal_weight.insert(0, "1")
        self.cal_weight.grid(row=3, column=4, sticky="w", padx=8)
        ctk.CTkLabel(bl, text="kg").grid(row=3, column=5, sticky="w")
        self.btn_cal = ctk.CTkButton(bl, text="Calibrate", command=self._on_calibrate)
        self.btn_cal.grid(row=3, column=6, padx=10)

        self.btn_esc1 = ctk.CTkButton(bl, text="Set ESC1 %", command=self._esc1_from_entry)
        self.btn_esc2 = ctk.CTkButton(bl, text="Set ESC2 %", command=self._esc2_from_entry)
        self.btn_esc1.grid(row=1, column=2, padx=10)
        self.btn_esc2.grid(row=1, column=3, padx=10)

        self.btn_cal_curr = ctk.CTkButton(bl, text="Calibrate Currents", command=self._on_calib_currents)
        self.btn_cal_curr.grid(row=1, column=6, padx=10, pady=(6,0))

        # --- E-STOP ---
        self.btn_estop = ctk.CTkButton(bl, text="E-STOP (Space)", command=self._on_estop,fg_color="#a00", hover_color="#b00")
        self.btn_estop.grid(row=1, column=7, padx=10)

        # --- Streaming controls (left control area) ---
        ctk.CTkLabel(bl, text="Stream period").grid(row=2, column=7, sticky="w", padx=8, pady=(8,2))
        self.stream_period = ctk.CTkEntry(bl, width=70)
        self.stream_period.insert(0, "120")  # default 120 ms (≈8.3 Hz)
        self.stream_period.grid(row=3, column=7, sticky="w", padx=8)
        ctk.CTkLabel(bl, text="ms").grid(row=3, column=8, sticky="w")

        self._streaming = False
        self.btn_stream_toggle = ctk.CTkButton(bl, text="Start Stream", command=self._on_toggle_stream)
        self.btn_stream_toggle.grid(row=3, column=9, padx=8, columnspan=2)


        # bottom-right COM panel
        br = ctk.CTkFrame(bottom)
        br.grid(row=0, column=1, sticky="e")

        ctk.CTkLabel(br, text="COM Port").grid(row=0, column=0, sticky="w", padx=8, pady=(8,2))
        self.port_var = ctk.StringVar(value=self._default_port())
        self.port_box = ctk.CTkComboBox(br, values=self._list_ports(), variable=self.port_var, width=120)
        self.port_box.grid(row=1, column=0, sticky="w", padx=8)

        ctk.CTkLabel(br, text="Baud Rate").grid(row=2, column=0, sticky="w", padx=8, pady=(8,2))
        self.baud_var = ctk.StringVar(value="921600")
        self.baud_box = ctk.CTkComboBox(br, values=["921600","460800","115200"], variable=self.baud_var, width=120)
        self.baud_box.grid(row=3, column=0, sticky="w", padx=8)

        self.status_var = ctk.StringVar(value="Disconnected")
        ctk.CTkLabel(br, text="Status").grid(row=0, column=1, sticky="w", padx=8, pady=(8,2))
        self.status_lbl = ctk.CTkLabel(br, textvariable=self.status_var, fg_color="#2ea043", corner_radius=4, width=110)
        self.status_lbl.grid(row=1, column=1, sticky="w", padx=8)

        ctk.CTkButton(br, text="Ping", command=self.link.send_ping, width=90).grid(row=1, column=2, padx=8)
        ctk.CTkButton(br, text="Connect", command=self._on_connect, width=90).grid(row=3, column=1, padx=8, pady=(8,2))
        ctk.CTkButton(br, text="Disconnect", command=self._on_disconnect, width=90).grid(row=3, column=2, padx=8, pady=(8,2))

        # start the queue pump
        self.after(30, self._drain_queue)
        self._print_help()

        # load persisted calibration (if any)
        self._load_calibration()

        # bind space for E-STOP
        self.bind_all("<space>", self._on_estop)

    # ---------- Terminal commands ----------
    def _handle_command(self, s: str):
        parts = s.strip().split()
        if not parts: return
        cmd = parts[0].lower()
        if cmd == "/help":
            self._print_help()
        elif cmd == "/ports":
            ports = self._list_ports()
            self._append_log("# Ports: " + (", ".join(ports) if ports else "<none>"))
        elif cmd == "/connect":
            port = parts[1] if len(parts) >= 2 else self.port_var.get().strip()
            baud = int(parts[2]) if len(parts) >= 3 else int(self.baud_var.get())
            if not port: self._append_log("! No COM port specified")
            else:
                self.port_var.set(port); self.baud_var.set(str(baud))
                self.link.open(port, baud)
        elif cmd == "/disconnect":
            self.link.close()
        elif cmd == "/ping":
            self.link.send_ping()
        elif cmd == "/snapshot":
            reqid = int(parts[1]) & 0xFF if len(parts) >= 2 else self._next_reqid()
            self.link.send_snapshot(reqid=reqid)
        elif cmd == "/hex":
            if len(parts) < 2: self._append_log("! usage: /hex <hex-bytes>")
            else: self.link.send_hex(" ".join(parts[1:]))
        elif cmd == "/stream":
            if len(parts) >= 2 and parts[1].lower() in ("on","start"):
                per = int(parts[2]) if len(parts) >= 3 else 100
                self.link.send_stream(per)
            elif len(parts) >= 2 and parts[1].lower() in ("off","stop"):
                self.link.send_stream(0)
            else:
                self._append_log("! usage: /stream on [period_ms] | /stream off")
        elif cmd == "/clear":
            self.log.delete("1.0", "end")
        elif cmd == "/esc":
            if len(parts) < 3:
                self._append_log("! usage: /esc <ch:1|2> <us>")
            else:
                self.link.send_set_esc(int(parts[1]), int(parts[2]))

        elif cmd == "/escpct":
            if len(parts) < 3:
                self._append_log("! usage: /escpct <ch:1|2> <percent 0..100>")
            else:
                ch = int(parts[1]); pct = float(parts[2])
                pct = max(0.0, min(100.0, pct))
                us = int(1050 + (1940-1050) * (pct/100.0))
                self.link.send_set_esc(ch, us)
        elif cmd == "/estop":
            self._on_estop()
        elif cmd == "/calib_curr":
            self.link.send_calib_currents()
        else:
            self._append_log(f"! unknown command: {cmd} (try /help)")

    # ---------- Event pump ----------
    def _drain_queue(self):
        try:
            # process up to N items per tick
            for _ in range(100):
                evt = self.q.get_nowait()
                typ = evt.get("type")
                if typ == "log":
                    self._append_log(evt["msg"])
                elif typ == "status":
                    self.status_var.set(evt["msg"])
                    self._append_log(f"# {evt['msg']}")
                elif typ == "sample":
                    data = evt.get("msg", {})
                    host_ms  = data.get("host_ms", int(time.time()*1000))
                    load_raw = data.get("load_raw", None)
                    dev_ms   = data.get("dev_ms", None)

                    vin1 = data.get("vin1_mv")
                    if vin1 is not None:
                        self.m1_voltage.set(f"{vin1/1000:.2f}")   # Motor 1 Voltage label

                    vin2 = data.get("vin2_mv")
                    if vin2 is not None:
                        self.m2_voltage.set(f"{vin2/1000:.2f}")   # Motor 2 Voltage label

                    curr1 = data.get("curr1_ma")
                    if curr1 is not None:
                        self.m1_current.set(f"{curr1/1000:.2f}")   # Motor 1 Current label
                    
                    curr2 = data.get("curr2_ma")
                    if curr2 is not None:
                        self.m2_current.set(f"{curr2/1000:.2f}")   # Motor 2 Current label

                    # collect for tare/cal
                    if self._cap_mode is not None and load_raw is not None:
                        self._cap_samples.append(float(load_raw))
                        self._finish_capture_if_ready(host_ms)

                    # live conversion to kg (if we have tare + scale)
                    kg = None if load_raw is None else self._apply_calibration(load_raw)

                    # ---- PLOT IN KG ----
                    if kg is not None:
                        self.plot.add_sample(host_ms, kg)
                        self.total_thrust.set(f"{kg:.3f}")  # update the Totals panel in kg

                    # show raw if you like
                    if load_raw is not None:
                        self.load_raw_var.set(str(load_raw))

                    # ---------- NEW: update RPM + throttle labels ----------
                    rpm1 = data.get("rpm1")
                    if rpm1 is not None:
                        self.m1_speed.set(str(rpm1))

                    rpm2 = data.get("rpm2")
                    if rpm2 is not None:
                        self.m2_speed.set(str(rpm2))

                    e1us = data.get("esc1_us")
                    if e1us:
                        pct = max(0.0, min(100.0, (e1us - 1050) * 100.0 / (1940 - 1050)))
                        self.m1_throttle.set(f"{pct:.0f}")

                    e2us = data.get("esc2_us")
                    if e2us:
                        pct = max(0.0, min(100.0, (e2us - 1050) * 100.0 / (1940 - 1050)))
                        self.m2_throttle.set(f"{pct:.0f}")

                    
        except queue.Empty:
            pass
        finally:
            # run at ~16 Hz; plenty for UI, leaves time for large resizes
            self.after(60, self._drain_queue)

    # ---------- Helpers ----------
    def _enqueue(self, evt: dict): self.q.put(evt)
    def _append_log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log.insert("end", f"[{ts}] {msg}\n")
        self.log.see("end")
        # trim if too many lines
        try:
            lines = int(self.log.index("end-1c").split(".")[0])
            if lines > MAX_LOG_LINES:
                # delete oldest extra lines in one go
                self.log.delete("1.0", f"{lines - MAX_LOG_LINES}.0")
        except Exception:
            pass
            self.log.insert("end", f"[{ts}] {msg}\n"); self.log.see("end")
    def _next_reqid(self):
        self._reqid = (self._reqid + 1) & 0xFF
        if self._reqid == 0: self._reqid = 1
        return self._reqid

    def _default_port(self):
        ports = Link.list_ports()
        for p in ports:
            if "STLink" in p.description or "STMicroelectronics" in p.description:
                return p.device
        return ports[0].device if ports else ""
    def _list_ports(self): return [p.device for p in Link.list_ports()]
    def _on_connect(self):
        port = self.port_var.get().strip()
        baud = int(self.baud_var.get())
        if not port: self._append_log("! No COM port selected"); return
        self.link.open(port, baud)
    def _on_disconnect(self): self.link.close()
    def _print_help(self):
        self._append_log("# /help | /ports | /connect [COMx] [baud] | /disconnect | /ping | /snapshot [reqid] | /stream on [ms]|off | /hex <bytes> | /clear")
    def _not_implemented(self): self._append_log("! (todo)")

    def on_close(self):
        try: self.link.close()
        except: pass
        self.destroy()

    def _esc1_from_entry(self):
        try: pct = float(self.manual_thr.get())
        except: pct = 0.0
        pct = max(0.0, min(100.0, pct))
        us = int(1050 + (1940-1050) * (pct/100.0))
        self.link.send_set_esc(1, us)

    def _esc2_from_entry(self):
        try: pct = float(self.manual_thr.get())
        except: pct = 0.0
        pct = max(0.0, min(100.0, pct))
        us = int(1050 + (1940-1050) * (pct/100.0))
        self.link.send_set_esc(2, us)

    def _on_toggle_stream(self):
        self._streaming = not self._streaming
        if self._streaming:
            try:
                per = int(self.stream_period.get())
            except ValueError:
                per = 120
            per = max(10, min(1000, per))
            self.link.send_stream(per)
            self.btn_stream_toggle.configure(text="Stop Stream")
            self._append_log(f"# streaming ON @ {per} ms")
        else:
            self.link.send_stream(0)
            self.btn_stream_toggle.configure(text="Start Stream")
            self._append_log("# streaming OFF")

    def _on_estop(self, *_):
        self.link.send_set_esc(1, 1050)
        self.link.send_set_esc(2, 1050)
        self._append_log("# EMERGENCY STOP: both ESC -> 1050us")

    def _on_calib_currents(self):
        self.link.send_calib_currents()
        self._append_log("# Sent current zero calibration")

    def _on_tare(self):
        # start a short capture: 1500 ms or >= 20 samples
        self._start_capture(mode="tare", duration_ms=1500, min_samples=20)
        self._append_log("# Tare started — keep loadcell unloaded and steady")

    def _on_calibrate(self):
        # parse known mass (kg)
        try:
            mass_kg = float(self.cal_weight.get())
            if not (mass_kg > 0):
                raise ValueError
        except Exception:
            self._append_log("! Calibration weight must be a positive number (kg)")
            return
        self._cal_mass_kg = mass_kg  # store for finish step
        self._start_capture(mode="cal", duration_ms=2000, min_samples=25)
        self._append_log(f"# Calibration started — place {mass_kg} kg and hold steady")

    def _start_capture(self, mode: str, duration_ms: int, min_samples: int):
        self._cap_mode = mode
        self._cap_samples = []
        self._cap_deadline_ms = int(time.time()*1000) + max(200, duration_ms)
        self._cap_min_samples = max(5, min_samples)

    def _finish_capture_if_ready(self, now_ms: int):
        if self._cap_mode is None:
            return
        enough_time = now_ms >= self._cap_deadline_ms
        enough_points = len(self._cap_samples) >= self._cap_min_samples
        if not (enough_time or enough_points):
            return

        # compute stats
        vals = self._cap_samples
        mean = sum(vals)/len(vals)
        var = sum((v-mean)**2 for v in vals)/max(1, len(vals)-1)
        std = var**0.5

        if self._cap_mode == "tare":
            self.tare_offset = mean
            self._append_log(f"# Tare OK: offset={mean:.1f} counts (std={std:.1f}, n={len(vals)})")
            # If already calibrated, persist new tare + existing scale
            if self.kg_per_count is not None:
                self._save_calibration()

        elif self._cap_mode == "cal":
            if self.tare_offset is None:
                self._append_log("! Cal failed: tare not set yet")
            else:
                counts = mean - self.tare_offset
                if abs(counts) < 1e-6:
                    self._append_log("! Cal failed: zero delta vs tare (is the weight on?)")
                else:
                    self.kg_per_count = self._cal_mass_kg / counts
                    self._append_log(f"# Cal OK: kg_per_count={self.kg_per_count:.6e} (std={std:.1f}, n={len(vals)})")

                    self.plot.set_units(title="Thrust (kg)", y_label="kg")
                    self.plot.clear()  # optional: reset view after calibrating
                    self._save_calibration()

        # reset capture
        self._cap_mode = None
        self._cap_samples.clear()

    def _apply_calibration(self, raw):
        """Return (kg or None) using current tare/scale."""
        if self.tare_offset is None or self.kg_per_count is None:
            return None
        return (raw - self.tare_offset) * self.kg_per_count
    
    def _ensure_cal_dir(self):
        try:
            os.makedirs(CAL_DIR, exist_ok=True)
        except Exception as e:
            self._append_log(f"! Could not create {CAL_DIR}: {e}")

    def _load_calibration(self):
        """Load tare & scale from disk, if present."""
        try:
            if not os.path.isfile(CAL_FILE):
                self._append_log(f"# No calibration file yet (will create at first save)")
                return
            with open(CAL_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            if data.get("version") != CAL_VERSION:
                self._append_log("# Calibration file version mismatch; ignoring (will overwrite on next save)")
                return
            self.tare_offset  = float(data.get("tare_offset"))
            self.kg_per_count = float(data.get("kg_per_count"))
            self._append_log(f"# Loaded calibration: tare={self.tare_offset:.1f} counts, "
                            f"kg_per_count={self.kg_per_count:.6e}")
            # make plot show kg axis if we’re calibrated
            if self.kg_per_count is not None and self.tare_offset is not None:
                self.plot.set_units(title="Thrust (kg)", y_label="kg")
        except Exception as e:
            self._append_log(f"! Failed to load calibration: {e}")

    def _save_calibration(self):
        """Persist tare & scale to disk."""
        try:
            if self.tare_offset is None or self.kg_per_count is None:
                self._append_log("! Not saving: tare and calibration not both set yet")
                return
            self._ensure_cal_dir()
            data = {
                "version": CAL_VERSION,
                "saved_at_epoch_ms": int(time.time()*1000),
                "tare_offset": float(self.tare_offset),
                "kg_per_count": float(self.kg_per_count),
                "units": "kg"   # stored as kg for thrust
            }
            with open(CAL_FILE, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            self._append_log(f"# Saved calibration to {CAL_FILE}")
        except Exception as e:
            self._append_log(f"! Failed to save calibration: {e}")

    def _reset_calibration(self):
        """Clear current calibration in memory (and on disk)."""
        self.tare_offset = None
        self.kg_per_count = None
        try:
            if os.path.isfile(CAL_FILE):
                os.remove(CAL_FILE)
                self._append_log("# Calibration reset (file removed)")
            else:
                self._append_log("# Calibration reset (no file to remove)")
        except Exception as e:
            self._append_log(f"! Failed to remove calibration file: {e}")
        # Optional: reset plot label (keeps the data)
        self.plot.set_units(title="Thrust (kg)", y_label="kg")

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
