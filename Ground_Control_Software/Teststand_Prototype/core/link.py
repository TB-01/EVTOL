# core/link.py
# Background serial I/O + frame parser. Emits events to the UI.

import time, threading
import serial, serial.tools.list_ports

from .protocol import (
    # IDs & types
    START, DEV_PC, DEV_MCU,
    MT_PING, MT_COMMAND, MT_COMMAND_ACK, MT_TEL_A,
    # TLVs
    TLV_TS_MS, TLV_TS_HOST_MS, TLV_CMD_CODE, TLV_CMD_RESULT, TLV_LOAD_RAW,
    TLV_STREAM_PERIOD_MS,
    TLV_ESC_CH, TLV_ESC_US, TLV_ESC1_US, TLV_ESC2_US,
    TLV_ESC_RPM1, TLV_ESC_RPM2,
    TLV_IC_IRQS, TLV_IC_PERIOD_US,
    TLV_VIN1_MV, TLV_VIN2_MV,
    TLV_I1_MA, TLV_I2_MA,
    # Commands
    CMD_SNAPSHOT, CMD_STREAM, CMD_SET_ESC, CMD_CALIB_I_ZERO,
    # helpers
    pack_frame, tlv_u32, tlv_u64, parse_tlvs, u32_le, i32_le, crc32c, build_cmd
)

class Link:
    """
    Serial link with background RX parser.
    on_event(dict) is called from the RX thread, but the UI should re-post it to Tk via a queue.
    Event dicts:
      {"type": "status"/"log"/"frame", ...}
    """
    def __init__(self, on_event):
        self.on_event = on_event
        self.ser = None
        self.rx_thread = None
        self.stop_event = threading.Event()
        self.seq = 0
        self.lock = threading.Lock()  # serialize writes

    # ---- lifecycle ----
    @staticmethod
    def list_ports():
        return list(serial.tools.list_ports.comports())

    def open(self, port: str, baud: int = 921600, timeout=0.2):
        if self.ser and self.ser.is_open:
            self._emit("status", f"Already open ({self.ser.port})")
            return
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
            self.stop_event.clear()
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
            self._emit("status", f"Opened {port} @ {baud}")
        except Exception as e:
            self.ser = None
            self._emit("status", f"Open failed: {e}")

    def close(self):
        self.stop_event.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.5)
        if self.ser:
            try: self.ser.close()
            except: pass
            self.ser = None
        self._emit("status", "Closed")

    # ---- TX helpers ----
    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        if self.seq == 0: self.seq = 1
        return self.seq

    def send_ping(self):
        if not self._ready(): return
        host_ms = int(time.time()*1000)
        payload = tlv_u64(TLV_TS_HOST_MS, host_ms)
        frm = pack_frame(DEV_MCU, DEV_PC, MT_PING, payload, reqid=0, seq=self._next_seq())
        self._write(frm)
        self._emit("log", f"> PING host_ms={host_ms} (len={len(frm)})")

    def send_snapshot(self, reqid=1):
        if not self._ready(): return
        payload = tlv_u32(TLV_CMD_CODE, CMD_SNAPSHOT)
        frm = pack_frame(DEV_MCU, DEV_PC, MT_COMMAND, payload, reqid=reqid, seq=self._next_seq())
        self._write(frm)
        self._emit("log", f"> CMD_SNAPSHOT reqid={reqid} (len={len(frm)})")

    def send_hex(self, hex_str: str):
        """Send raw bytes given as hex string '45 00 01...' (for lab work)."""
        if not self._ready(): return
        try:
            b = bytes.fromhex(hex_str)
            self._write(b)
            self._emit("log", f"> RAW {len(b)}B: {hex_str}")
        except ValueError:
            self._emit("log", "! invalid hex")

    

    def send_stream(self, period_ms: int):
        """Start (period_ms>0) or stop (0) streaming telemetry from MCU."""
        if not self._ready(): return
        payload = build_cmd(CMD_STREAM, tlv_u32(TLV_STREAM_PERIOD_MS, max(0, int(period_ms))))
        frm = pack_frame(DEV_MCU, DEV_PC, MT_COMMAND, payload, reqid=3, seq=self._next_seq())
        self._write(frm)
        self._emit("log", f"> CMD_STREAM period_ms={period_ms} (len={len(frm)})")

    def send_set_esc(self, ch: int, us: int, reqid: int = 5):
        if not self._ready(): return
        ch = 1 if ch == 1 else 2
        payload = build_cmd(CMD_SET_ESC,
                            tlv_u32(TLV_ESC_CH, ch),
                            tlv_u32(TLV_ESC_US, max(800, min(2200, int(us)))))
        frm = pack_frame(DEV_MCU, DEV_PC, MT_COMMAND, payload, reqid=reqid, seq=self._next_seq())
        self._write(frm)
        self._emit({"type":"log","msg":f"> CMD_SET_ESC ch={ch} us={us} (len={len(frm)})"})

    def send_calib_currents(self, mask: int = 0x3, reqid: int = 7):
        if not self._ready(): return
        
        payload = build_cmd(CMD_CALIB_I_ZERO)        # <-- no payload TLVs
        frm = pack_frame(DEV_MCU, DEV_PC, MT_COMMAND, payload, reqid=reqid, seq=self._next_seq())
        self._write(frm)
        self._emit("log", f"> CMD_CALIB_I_ZERO (len={len(frm)})")

    # ---- RX loop & parser ----
    def _rx_loop(self):
        while not self.stop_event.is_set():
            try:
                b = self.ser.read(1) if self.ser else b""
                if not b:
                    continue
                if b[0] != START:
                    continue
                lb = self.ser.read(2)
                if len(lb) < 2:
                    self._emit("log", "! short length")
                    continue
                L = int.from_bytes(lb, "little")
                body = self.ser.read(L-3)
                if len(body) != (L-3):
                    self._emit("log", "! short body")
                    continue
                frm = bytes([START]) + lb + body
                if crc32c(frm[:-4]) != int.from_bytes(frm[-4:], "little"):
                    self._emit("log", "! CRC fail")
                    continue
                self._handle_frame(frm)
            except Exception as e:
                self._emit("status", f"RX error: {e}")
                time.sleep(0.1)

    def _handle_frame(self, frm: bytes):
        hdr = frm[3:11]
        rx_id, tx_id, mtype, ver, flags, seq, reqid, rsv = hdr
        payload = frm[11:-4]
        tlvs = parse_tlvs(payload)
        now_ms = int(time.time()*1000)

        if mtype == MT_PING:
            host_echo = int.from_bytes(tlvs.get(TLV_TS_HOST_MS, b"\0"*8), "little")
            dev_ms    = u32_le(tlvs.get(TLV_TS_MS, None), 0)
            rtt = now_ms - host_echo if host_echo else None
            self._emit("log", f"< PONG rtt={rtt} ms dev_uptime={dev_ms} ms seq={seq}")

        elif mtype == MT_COMMAND_ACK:
            code = u32_le(tlvs.get(TLV_CMD_CODE, None), 0)
            res  = u32_le(tlvs.get(TLV_CMD_RESULT, None), 0)
            self._emit("log", f"< ACK reqid={reqid} code=0x{code:08X} result={res} seq={seq}")

        elif mtype == MT_TEL_A:
            ts  = u32_le(tlvs.get(TLV_TS_MS, None), 0)
            raw = i32_le(tlvs.get(TLV_LOAD_RAW, b""))
            e1  = u32_le(tlvs.get(TLV_ESC1_US, None), 0)
            e2  = u32_le(tlvs.get(TLV_ESC2_US, None), 0)
            rpm1 = u32_le(tlvs.get(TLV_ESC_RPM1, None), 0)
            rpm2 = u32_le(tlvs.get(TLV_ESC_RPM2, None), 0)
            # log (rpm only if present)
            rlog = f" rpm1={rpm1}" if rpm1 else ""
            rlog += f" rpm2={rpm2}" if rpm2 else ""
            ic_irqs = u32_le(tlvs.get(TLV_IC_IRQS, None), 0)
            ic_per  = u32_le(tlvs.get(TLV_IC_PERIOD_US, None), 0)

            vin1 = u32_le(tlvs.get(TLV_VIN1_MV, None), 0)
            vin2 = u32_le(tlvs.get(TLV_VIN2_MV, None), 0)

            raw_pa3 = u32_le(tlvs.get(0x90, None), 0)
            raw_pa4 = u32_le(tlvs.get(0x91, None), 0)
            raw_VREFINT = u32_le(tlvs.get(0x92, None), 0)

            curr1 = i32_le(tlvs.get(TLV_I1_MA, b""))
            curr2 = i32_le(tlvs.get(TLV_I2_MA, b""))

            self._emit({"type":"log","msg":f"< TEL_A ts_ms={ts} load_raw={raw} esc1={e1}us esc2={e2}us "
                                            f"vin1={vin1}mV vin2={vin2}mV rpm1={rpm1} ic_irqs={ic_irqs} per_us={ic_per} seq={seq} raw_pa3={raw_pa3} raw_pa4={raw_pa4} raw_VREFINT={raw_VREFINT}{rlog} curr1={curr1}mA curr2={curr2}mA"})

            # emit a structured sample for the UI
            msg = {"host_ms": now_ms, "dev_ms": ts, "load_raw": raw}
            if e1:   msg["esc1_us"] = e1
            if e2:   msg["esc2_us"] = e2
            if rpm1: msg["rpm1"]    = rpm1
            if rpm2: msg["rpm2"]    = rpm2
            msg["vin1_mv"] = vin1
            msg["vin2_mv"] = vin2
            msg["curr1_ma"] = curr1
            msg["curr2_ma"] = curr2
            self._emit({"type":"sample","msg": msg})


        else:
            self._emit("log", f"< frame type=0x{mtype:02X} len={len(payload)} seq={seq}")

    # ---- utils ----
    def _ready(self):
        if not self.ser or not self.ser.is_open:
            self._emit("status","Port not open")
            return False
        return True

    def _write(self, b: bytes):
        with self.lock:
            self.ser.write(b)

    def _emit(self, typ, msg=None):
        """Emit an event to the UI.

        Supports both styles:
        _emit("log", "text")
        _emit({"type":"log","msg":"text"})
        """
        try:
            if msg is None and isinstance(typ, dict):
                evt = typ
            else:
                evt = {"type": typ, "msg": msg}
            self.on_event(evt)
        except Exception:
            pass
