"""
Helper functions for interacting with the CAN-A protocol across the Waveshare adapter.
"""

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

def encode_for_waveshare(can_id: int, data: bytes) -> bytes:
    """
    Encode a CAN frame in Waveshare USB-CAN-A binary format.

    Always produces extended (29-bit) frames since SPARK MAX requires them.
    The type byte is built as:
      0xC0          — top two bits set (frame marker)
      | (1 << 5)    — extended ID flag
      | data_length — number of data bytes (0-8)
    The CAN ID is packed little-endian across 4 bytes, with only the lower
    29 bits used (top 3 bits of byte 3 are masked to 0x1F).
    """
    data_length = len(data)

    type_byte = (
        0xC0          |    # frame marker: top two bits always set
        (1 << 5)      |    # extended 29-bit CAN ID flag
        data_length        # how many data bytes follow
    )

    # CAN ID packed as 4 bytes, least significant byte first (little-endian).
    # The final byte is masked to 0x1F because only 29 bits are used —
    # 3 full bytes (24 bits) plus 5 bits of the fourth byte.
    can_id_byte_0 = (can_id >>  0) & 0xFF
    can_id_byte_1 = (can_id >>  8) & 0xFF
    can_id_byte_2 = (can_id >> 16) & 0xFF
    can_id_byte_3 = (can_id >> 24) & 0x1F

    packet = bytearray([
        0xAA,          # start-of-frame marker
        type_byte,
        can_id_byte_0,
        can_id_byte_1,
        can_id_byte_2,
        can_id_byte_3,
    ])
    packet += bytes(data)
    packet.append(0x55)    # end-of-frame marker

    return bytes(packet)


def read_waveshare_frame_from_serial(serial_port) -> tuple | None:
    """
    Read and decode one Waveshare frame from the serial port.

    Reads byte-by-byte, looking for 0xAA start marker. Returns
    (can_id, data) for a valid CAN data frame, or None if the read
    timed out, was a config packet, or was malformed.

    Config packets start with AA 55 12 — these are consumed and
    discarded since the simulator doesn't need to act on them.
    """
    first_byte = serial_port.read(1)
    if not first_byte or first_byte[0] != 0xAA:
        return None    # timed out or not a frame start

    type_byte = serial_port.read(1)
    if not type_byte:
        return None

    # Config packet: starts AA 55 12, total 20 bytes.
    # The plugin sends this once on startup to configure the adapter.
    if type_byte[0] == 0x55:
        third_byte = serial_port.read(1)
        if third_byte and third_byte[0] == 0x12:
            serial_port.read(17)    # discard remaining 17 bytes of the 20-byte packet
            print("Config packet received")
        return None

    # Type byte must have top two bits set (0b11xxxxxx)
    if (type_byte[0] & 0xC0) != 0xC0:
        return None

    is_extended = bool(type_byte[0] & 0x20)    # bit 5: extended 29-bit ID
    data_length  = type_byte[0] & 0x0F         # bits 3-0: number of data bytes

    id_bytes = serial_port.read(4 if is_extended else 2)
    data     = serial_port.read(data_length)
    serial_port.read(1)    # discard 0x55 end-of-frame byte

    if not is_extended:
        return None    # plugin only sends extended frames

    # Reconstruct the 29-bit CAN ID from 4 little-endian bytes.
    # The top byte is masked to 0x1F to discard the 3 unused bits.
    can_id = (
            id_bytes[0]
            | (id_bytes[1] <<  8)
            | (id_bytes[2] << 16)
            | ((id_bytes[3] & 0x1F) << 24)
    )

    return can_id, bytes(data)