# =============================================================================
# Waveshare USB-CAN-A binary protocol
# =============================================================================
#
# The Waveshare adapter uses a simple binary framing over UART:
#
#   [0xAA] [type] [id bytes] [data bytes] [0x55]
#
#   0xAA — start-of-frame marker
#   type byte:
#     bits 7-6  always 0b11 (0xC0) — identifies this as a data frame
#     bit  5    1 = extended 29-bit CAN ID, 0 = standard 11-bit
#     bit  4    1 = RTR (remote request), 0 = normal data frame
#     bits 3-0  DLC: number of data bytes (0-8)
#   id bytes:
#     4 bytes little-endian for extended IDs (only top 29 bits used)
#     2 bytes little-endian for standard IDs
#   data bytes: DLC bytes of CAN payload
#   0x55 — end-of-frame marker
#
# Config packet (sent once by the plugin on startup, 20 bytes):
#   [0xAA] [0x55] [0x12] [baud_code] [filter_type]
#   [filter_id 4 bytes LE] [block_id 4 bytes LE]
#   [mode] [disable_retransmit] [0x00 * 4] [checksum]
#
# =============================================================================

def encode_for_waveshare(can_id, data):
    """
    Encode a CAN frame in Waveshare USB-CAN-A binary format.

    Always produces extended (29-bit) frames since SPARK MAX requires them.
    The type byte is built as:
      0xC0          — top two bits set (frame marker)
      | (1 << 5)    — extended ID flag
      | dlc         — number of data bytes
    The CAN ID is packed little-endian across 4 bytes, with only the lower
    29 bits used (top 3 bits of byte 3 are masked to 0x1F).
    """
    dlc = len(data)
    pkt = bytearray([
        0xAA,                          # start-of-frame
        0xC0 | (1 << 5) | dlc,         # type = extended data frame, DLC
    ])
    # CAN ID as 4 bytes little-endian, top 3 bits of last byte unused
    pkt += bytes([can_id >> s & m for s, m in [(0,0xFF),(8,0xFF),(16,0xFF),(24,0x1F)]])
    pkt += bytes(data)                 # CAN payload
    pkt.append(0x55)                   # end-of-frame
    return bytes(pkt)

def read_waveshare_frame_from_serial(ser):
    """
    Read and decode one Waveshare frame from the serial port.

    Reads byte-by-byte, looking for 0xAA start marker. Returns
    (can_id, data) for a valid CAN data frame, or None if the byte
    timed out, was a config packet, or was malformed.

    Config packets start with AA 55 12 — these are consumed and
    discarded since the simulator doesn't need to act on them.
    """
    b = ser.read(1)
    if not b or b[0] != 0xAA:         # no data or not a frame start
        return None

    hdr = ser.read(1)
    if not hdr:
        return None

    # Config packet detection: second byte 0x55, third byte 0x12
    if hdr[0] == 0x55:
        rest = ser.read(1)
        if rest and rest[0] == 0x12:
            ser.read(17)               # consume remaining 17 bytes of 20-byte packet
            print("Config packet received")
        return None

    # Validate frame marker bits (must be 0b11xxxxxx)
    if (hdr[0] & 0xC0) != 0xC0:
        return None

    extended = bool(hdr[0] & 0x20)    # bit 5: extended ID flag
    dlc      = hdr[0] & 0x0F          # bits 3-0: data length

    id_bytes = ser.read(4 if extended else 2)
    data     = ser.read(dlc)
    ser.read(1)                        # consume 0x55 end-of-frame byte

    if not extended:
        return None                    # plugin only sends extended frames

    # Reconstruct 29-bit CAN ID from 4 little-endian bytes
    # Mask top byte to 0x1F (29 bits = 3 full bytes + 5 bits of 4th byte)
    can_id = (
            id_bytes[0]              |
            (id_bytes[1] <<  8)      |
            (id_bytes[2] << 16)      |
            ((id_bytes[3] & 0x1F) << 24)
    )
    return can_id, bytes(data)