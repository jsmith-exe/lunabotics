#!/usr/bin/env python3
"""
Parse a socat -x -v CAN traffic log into decoded CAN frames.

socat -x -v produces two interleaved sections per transfer:
  - A header line:  > 2026/08/15 17:46:40.200436  length=29 from=0 to=28
  - Hex dump lines: aa 55 12 01 ...  .U......
  - A "--" separator between transfers

Direction:
  >  data arriving FROM the adapter (SPARK MAX status frames)
  <  data sent TO the adapter (plugin commands, config, heartbeats)

Usage:
  python3 parse_can_log.py raw_can_log.txt
"""

import re
import sys
import struct
from dataclasses import dataclass

def decode_frc_id(can_id: int) -> dict:
    """Unpack a 29-bit FRC extended CAN ID into its named fields."""
    return {
        "device_type":  (can_id >> 24) & 0x1F,
        "manufacturer": (can_id >> 16) & 0xFF,
        "api_class":    (can_id >> 10) & 0x3F,
        "api_index":    (can_id >>  6) & 0x0F,
        "device_id":     can_id        & 0x3F,
    }

# Known api_class + api_index combinations for SPARK MAX (device_type=2, manufacturer=5)
KNOWN_FRAMES = {
    (0,  2): "Duty Cycle Set",
    (1,  2): "Velocity Set (RPM)",
    (6,  0): "Clear Faults",
    (9,  2): "Heartbeat (roboRIO)",
    (11, 2): "Heartbeat (non-RIO)",
    (46, 0): "Status 0 — Applied Output",
    (46, 1): "Status 1 — Velocity (RPM)",
    (46, 2): "Status 2 — Position (rotations)",
    (46, 3): "Status 3 — Analog Voltage",
}

HEARTBEAT_ID = (2 << 24) | (5 << 16) | (11 << 10) | (2 << 6) | 0

@dataclass
class CANFrame:
    direction: str          # ">" = from adapter, "<" = to adapter
    timestamp: str
    can_id: int
    extended: bool
    remote: bool
    dlc: int
    data: bytes
    label: str = ""         # human-readable frame type
    value: str = ""         # decoded payload where known

def decode_frames_from_bytes(raw: bytes, direction: str, timestamp: str) -> list[CANFrame]:
    """
    Walk a raw byte buffer and extract all Waveshare frames.

    Each frame: [0xAA][type][id 4B LE][data 0-8B][0x55]
    type byte: bits 7-6 = 0b11 (frame marker)
               bit  5   = extended ID flag
               bit  4   = RTR flag
               bits 3-0 = DLC
    Config packets (AA 55 12 ...) are 20 bytes and decoded separately.
    """
    frames = []
    i = 0
    while i < len(raw):
        if raw[i] != 0xAA:
            i += 1
            continue

        if i + 1 >= len(raw):
            break

        type_byte = raw[i + 1]

        # Config packet: AA 55 12 ...
        if type_byte == 0x55 and i + 2 < len(raw) and raw[i + 2] == 0x12:
            if i + 20 <= len(raw):
                baud_code = raw[i + 3]
                frames.append(CANFrame(
                    direction=direction, timestamp=timestamp,
                    can_id=0, extended=False, remote=False, dlc=0, data=b"",
                    label=f"Config packet (baud_code=0x{baud_code:02X})",
                ))
            i += 20
            continue

        if (type_byte & 0xC0) != 0xC0:
            i += 1
            continue

        extended = bool(type_byte & 0x20)
        remote   = bool(type_byte & 0x10)
        dlc      = type_byte & 0x0F
        id_len   = 4 if extended else 2
        total    = 1 + 1 + id_len + (0 if remote else dlc) + 1

        if i + total > len(raw):
            break

        if raw[i + total - 1] != 0x55:
            i += 1
            continue

        id_bytes = raw[i + 2: i + 2 + id_len]
        if extended:
            can_id = (id_bytes[0] | id_bytes[1] << 8 | id_bytes[2] << 16
                      | (id_bytes[3] & 0x1F) << 24)
        else:
            can_id = id_bytes[0] | (id_bytes[1] & 0x07) << 8

        data = raw[i + 2 + id_len: i + 2 + id_len + dlc] if not remote else b""

        label, value = describe(can_id, data, extended)

        frames.append(CANFrame(
            direction=direction, timestamp=timestamp,
            can_id=can_id, extended=extended, remote=remote,
            dlc=dlc, data=data, label=label, value=value,
        ))
        i += total

    return frames


def describe(can_id: int, data: bytes, extended: bool) -> tuple[str, str]:
    """Return a (label, decoded_value) pair for a CAN frame."""
    if not extended:
        return "Standard frame", ""

    f = decode_frc_id(can_id)
    if f["device_type"] != 2 or f["manufacturer"] != 5:
        return f"Unknown device (type={f['device_type']} mfr={f['manufacturer']})", ""

    key = (f["api_class"], f["api_index"])
    label = KNOWN_FRAMES.get(key, f"api_class={f['api_class']} api_index={f['api_index']}")
    label += f" | device_id={f['device_id']}"

    value = ""
    if key in {(0, 2), (1, 2)} and len(data) >= 4:
        # Duty cycle or velocity: float32 LE setpoint
        v = struct.unpack_from("<f", data)[0]
        value = f"{v:.4f} {'(duty)' if key == (0,2) else 'RPM'}"
    elif key == (46, 0) and len(data) >= 2:
        # Status 0: applied output as int16 / 32767
        v = struct.unpack_from("<h", data)[0] / 32767.0
        value = f"{v:.4f} (applied output)"
    elif key == (46, 1) and len(data) >= 4:
        # Status 1: velocity RPM as float32
        v = struct.unpack_from("<f", data)[0]
        value = f"{v:.2f} RPM"
    elif key == (46, 2) and len(data) >= 4:
        # Status 2: position rotations as float32
        v = struct.unpack_from("<f", data)[0]
        value = f"{v:.4f} rotations"
    elif key == (46, 3) and len(data) >= 2:
        # Status 3: analog voltage in 2Q8 fixed point (voltage * 256)
        raw_adc = struct.unpack_from("<H", data)[0]
        value = f"{raw_adc / 256:.4f} V (analog)"
    elif can_id == HEARTBEAT_ID:
        value = "0xFF * 8 (broadcast)"

    return label, value

HEADER_RE = re.compile(
    r'^([<>])\s+(\d{4}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}\.\d+)\s+length=(\d+)'
)
HEX_RE = re.compile(r'^\s+([0-9a-f ]{48})', re.IGNORECASE)

def parse_socat_log(path: str) -> list[CANFrame]:
    """
    Read a socat -x -v log file and return a flat list of decoded CANFrames.

    The log alternates between:
      - Transfer headers  (direction, timestamp, byte count)
      - Hex dump lines    (16 bytes per line, left-hand column only)
      - "--" separators
    """
    all_frames = []
    direction = ""
    timestamp = ""
    current_bytes = bytearray()

    def flush():
        if current_bytes and direction:
            all_frames.extend(
                decode_frames_from_bytes(bytes(current_bytes), direction, timestamp)
            )

    with open(path) as f:
        for line in f:
            line = line.rstrip()

            if line == "--":
                flush()
                current_bytes.clear()
                continue

            m = HEADER_RE.match(line)
            if m:
                flush()
                current_bytes.clear()
                direction = m.group(1)
                timestamp = m.group(2)
                continue

            m = HEX_RE.match(line)
            if m:
                hex_part = m.group(1).strip()
                current_bytes.extend(bytes.fromhex(hex_part.replace(" ", "")))

    flush()
    return all_frames

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "raw_can_log.txt"
    frames = parse_socat_log(path)

    arrow = {"<": "→ adapter", ">": "← adapter"}

    for f in frames:
        if f.label.startswith("Config"):
            print(f"[{f.timestamp}] {arrow.get(f.direction, f.direction):10s}  {f.label}")
        else:
            hex_data = f.data.hex(" ") if f.data else ""
            print(
                f"[{f.timestamp}] {arrow.get(f.direction, f.direction):10s}"
                f"  ID=0x{f.can_id:08X}  {f.label}"
                + (f"  =  {f.value}" if f.value else "")
                + (f"  [{hex_data}]" if hex_data else "")
            )

if __name__ == "__main__":
    main()