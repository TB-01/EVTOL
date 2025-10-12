import serial, struct, time

PORT = "COM10"    # set your COM port
BAUD = 921600

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
MT_TEL_A        = 0x65  # telemetry frame "A" (you’ll extend with more TLVs)

# ========= TLV Types =========
# NOTE: “size” refers to the Value field length on the wire.
TLV_TS_MS       = 0x01  # u32  device uptime in ms (HAL_GetTick)
TLV_TS_HOST_MS  = 0x02  # u64  host epoch time in ms (sent by PC, echoed by MCU)
TLV_CMD_CODE    = 0x4F  # u32  which command (see CMD_* below)
TLV_CMD_RESULT  = 0x50  # u32  0=OK, non-zero=error
TLV_LOAD_RAW    = 0x30  # i32  raw HX711 counts (two’s complement)
# Optional diagnostics:
# TLV_HX_FLAGS  = 0x31  # u32  bit0 = DOUT idle level (1=high)

# ========= Command Codes =========
CMD_ECHO        = 0x00000000  # no-op; verify command/ack path
CMD_SNAPSHOT    = 0x00000001  # send one MT_TEL_A immediately


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if (crc & 1) else 0)
    return (~crc) & 0xFFFFFFFF

def tlv_u32(t, v): 
    return bytes([t,4,0, v & 0xFF, (v>>8)&0xFF, (v>>16)&0xFF, (v>>24)&0xFF])

def tlv_u64(t, v):
    return bytes([t,8,0]) + int(v).to_bytes(8, "little", signed=False)

def pack_frame(rx_id, tx_id, msg_type, payload=b"", flags=1, reqid=0):
    hdr = bytes([rx_id, tx_id, msg_type, COMM_VER, flags, 1, reqid, 0])
    core = bytes([START]) + struct.pack("<H", 1+2+len(hdr)+len(payload)+4) + hdr + payload
    return core + struct.pack("<I", crc32c(core))

def read_frame(ser):
    # seek start
    while True:
        b = ser.read(1)
        if not b: return None
        if b[0] != START: continue
        Lb = ser.read(2)
        if len(Lb) < 2: return None
        L = int.from_bytes(Lb, "little")
        body = ser.read(L-3)
        if len(body) != L-3: return None
        frm = bytes([START]) + Lb + body
        if crc32c(frm[:-4]) != int.from_bytes(frm[-4:], "little"):
            print("CRC fail")
            continue
        return frm

def parse_tlvs(payload: bytes):
    i, out = 0, {}
    while i + 3 <= len(payload):
        t = payload[i]; l = payload[i+1] | (payload[i+2] << 8); i += 3
        if i + l > len(payload): break
        v = payload[i:i+l]; i += l
        out[t] = v
    return out

def send_command_echo(ser, reqid=1):
    tlv = tlv_u32(TLV_CMD_CODE, CMD_ECHO)
    ser.write(pack_frame(DEV_MCU, DEV_PC, MT_COMMAND, tlv, flags=1, reqid=reqid))

def send_snapshot(ser, reqid=7):
    tlv = tlv_u32(TLV_CMD_CODE, CMD_SNAPSHOT)
    ser.write(pack_frame(DEV_MCU, DEV_PC, MT_COMMAND, tlv, flags=1, reqid=reqid))

def u32_le(b): return int.from_bytes(b, "little", signed=False)


with serial.Serial(PORT, BAUD, timeout=0.4) as ser:
    # 1) optional: ping RTT sanity
    host_send_ms = int(time.time()*1000)
    payload = bytes([TLV_TS_HOST_MS,8,0]) + host_send_ms.to_bytes(8,"little")
    ser.write(pack_frame(DEV_MCU, DEV_PC, MT_PING, payload))
    frm = read_frame(ser)
    print("ping ok" if frm else "no ping reply")

    # 2) request a snapshot
    reqid = 99
    send_snapshot(ser, reqid=reqid)

    # 3) read until we see ACK and a TEL_A
    got_ack = got_tel = False
    while not (got_ack and got_tel):
        frm = read_frame(ser)
        if not frm:
            print("timeout waiting for frames")
            break
        hdr = frm[3:11]
        mtype, rx_reqid = hdr[2], hdr[6]
        tlvs = parse_tlvs(frm[11:-4])

        if mtype == MT_COMMAND_ACK:
            code = u32_le(tlvs.get(TLV_CMD_CODE, b"\0\0\0\0"))
            res  = u32_le(tlvs.get(TLV_CMD_RESULT, b"\0\0\0\0"))
            print(f"ACK: reqid={rx_reqid} code=0x{code:08X} result={res}")
            got_ack = True

        elif mtype == MT_TEL_A:
            tlvs = parse_tlvs(frm[11:-4])
            ts = int.from_bytes(tlvs.get(TLV_TS_MS, b"\0\0\0\0"), "little", signed=False)
            raw = int.from_bytes(tlvs.get(TLV_LOAD_RAW, b""), "little", signed=True) if TLV_LOAD_RAW in tlvs else None
            print(f"TEL_A: ts_ms={ts} load_raw={raw}")
            got_tel = True

        else:
            print("other frame type:", hex(mtype))