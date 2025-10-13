# core/protocol.py
# Wire constants + helpers (no UI/threading here).

import struct

# ========= Protocol constants (wire format) =========
START      = 0x45   # Start-of-frame byte (helps resync)
COMM_VER   = 0x01   # Protocol version
FINAL_FLAG = 0x01   # Header flags bitmask: 0x01 = FINAL (no fragments)

# ========= Device IDs =========
DEV_PC  = 0x0A      # Sender ID for host/PC
DEV_MCU = 0x20      # Sender ID for device/firmware

# ========= Message Types =========
MT_PING         = 0x05  # request/response for round-trip timing
MT_COMMAND      = 0x0A  # request: commands with TLV payload
MT_COMMAND_ACK  = 0x0B  # response: ack/nack to MT_COMMAND
MT_TEL_A        = 0x65  # telemetry frame "A"

# ========= TLV Types =========
# NOTE: “size” refers to the Value field length on the wire.
TLV_TS_MS       = 0x01  # u32  device uptime in ms (HAL_GetTick)
TLV_TS_HOST_MS  = 0x02  # u64  host epoch time in ms (sent by PC, echoed by MCU)
TLV_CMD_CODE    = 0x4F  # u32  which command (see CMD_* below)
TLV_CMD_RESULT  = 0x50  # u32  0=OK, non-zero=error
TLV_LOAD_RAW    = 0x30  # i32  raw HX711 counts (two’s complement)
TLV_STREAM_PERIOD_MS = 0x40 # u32  telemetry stream period in ms (for CMD_STREAM)

TLV_ESC_CH           = 0x60
TLV_ESC_US           = 0x61
TLV_ESC1_US          = 0x62
TLV_ESC2_US          = 0x63

TLV_ESC_RPM1 = 0x70
TLV_ESC_RPM2 = 0x71

TLV_IC_IRQS       = 0x72
TLV_IC_PERIOD_US  = 0x73

TLV_VIN1_MV = 0x80
TLV_VIN2_MV = 0x81

TLV_I1_MA  = 0x82
TLV_I2_MA  = 0x83


# TLV_HX_FLAGS  = 0x31  # u32  bit0 = DOUT idle level (optional diag)

# ========= Command Codes =========
CMD_ECHO        = 0x00000000  # no-op; verify command/ack path
CMD_SNAPSHOT    = 0x00000001  # send one MT_TEL_A immediately
CMD_STREAM      = 0x00000002  # start/stop periodic MT_TEL_A frames
CMD_SET_ESC     = 0x00000010  # set ESC pulse widths (for motor control)
CMD_CALIB_I_ZERO = 0x00000020  # calibrate current sensor zero (no currents)

# ========= Helpers =========
def build_cmd(cmd_code: int, *tlvs: bytes) -> bytes:
    return tlv_u32(TLV_CMD_CODE, cmd_code) + b"".join(tlvs)

def crc32c(data: bytes) -> int:
    """Tiny bitwise CRC-32C (Castagnoli). Small/portable; fast enough for our sizes."""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if (crc & 1) else 0)
    return (~crc) & 0xFFFFFFFF

def pack_frame(rx_id, tx_id, msg_type, payload=b"", flags=FINAL_FLAG, reqid=0, seq=1):
    hdr = bytes([rx_id, tx_id, msg_type, COMM_VER, flags, seq, reqid, 0])
    core = bytes([START]) + struct.pack("<H", 1+2+len(hdr)+len(payload)+4) + hdr + payload
    return core + struct.pack("<I", crc32c(core))

def tlv_u32(t, v): return bytes([t,4,0, v & 0xFF, (v>>8)&0xFF, (v>>16)&0xFF, (v>>24)&0xFF])
def tlv_u64(t, v): return bytes([t,8,0]) + int(v).to_bytes(8, "little", signed=False)

def parse_tlvs(payload: bytes):
    out, i = {}, 0
    while i + 3 <= len(payload):
        t = payload[i]; l = payload[i+1] | (payload[i+2] << 8); i += 3
        if i + l > len(payload): break
        out[t] = payload[i:i+l]
        i += l
    return out

def u32_le(b, default=0): return int.from_bytes(b if b else default.to_bytes(4,"little"), "little", signed=False)
def i32_le(b):            return int.from_bytes(b, "little", signed=True) if b else None
