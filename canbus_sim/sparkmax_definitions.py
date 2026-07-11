# =============================================================================
# SPARK MAX periodic status frame builders (firmware 24.x layout)
# =============================================================================
#
# SPARK MAXs broadcast these automatically at a configurable rate.
# The simulator sends them at 50 Hz to keep the plugin's feedback path active.
#
# api_class=6 (0x06) is the periodic status class for firmware 24.x.
# firmware 25+ uses api_class=46 (0x2E) with a different layout — not simulated here.
#
# =============================================================================
import struct
from can_definitions import *
from waveshare_definitions import *

def create_applied_output_status(dev_id, applied):
    """
    Periodic Status 0: applied output (what the motor is actually doing).

    CAN ID: frc_id(class=6, index=0, dev_id)
    Payload: bytes 0-1 = applied output as signed int16, scaled by 32767
             bytes 2-7 = unused (zeroed)

    applied is in [-1.0, 1.0]. Multiplying by 32767 maps it to the full
    int16 range. The plugin reads this back and divides by 32767 to recover
    the float.
    """
    raw = int(applied * 32767)
    return encode_for_waveshare(create_frc_id(6, 0, dev_id), struct.pack("<h", raw) + b"\x00" * 6)

def create_velocity_status(dev_id, rpm):
    """
    Periodic Status 1: encoder velocity.

    CAN ID: frc_id(class=6, index=1, dev_id)
    Payload: bytes 0-3 = motor velocity in RPM as IEEE 754 float32 little-endian
             bytes 4-7 = unused (zeroed)

    The plugin converts RPM to rad/s and divides by gear_ratio to get
    wheel velocity for ros2_control.
    """
    return encode_for_waveshare(create_frc_id(6, 1, dev_id), struct.pack("<f", rpm) + b"\x00" * 4)

def create_position_status(dev_id, pos):
    """
    Periodic Status 2: encoder position.

    CAN ID: frc_id(class=6, index=2, dev_id)
    Payload: bytes 0-3 = motor position in rotations as float32 little-endian
             bytes 4-7 = unused (zeroed)

    The plugin divides by gear_ratio to get wheel position in rotations,
    then multiplies by 2π for radians.
    """
    return encode_for_waveshare(create_frc_id(6, 2, dev_id), struct.pack("<f", pos) + b"\x00" * 4)

def create_analog_status(dev_id, voltage):
    """
    Periodic Status 3: analog sensor data (used for linear actuator feedback).

    CAN ID: 0x020518C0 + dev_id
      0x02051800 is the base for firmware 24 status frames (class=6 shifted),
      0xC0 adds api_index=3 (index 3 << 6 = 0xC0), giving Status 3.

    Payload: bytes 0-1 = ADC voltage in 2Q8 fixed-point little-endian
             bytes 2-7 = unused (zeroed)

    2Q8 fixed-point means 2 integer bits and 8 fractional bits.
    To encode: raw = voltage * 256 (shift left 8 bits).
    To decode: voltage = raw / 256.
    Range: 0 to ~255V representable, but SPARK MAX analog input is 0-3.3V.

    The plugin maps this voltage to actuator position using configured
    min/max voltage thresholds (default 0.279V to 1.85V).
    """
    raw = max(0, min(0xFFFF, int(voltage * 256)))
    return encode_for_waveshare(0x020518C0 + dev_id, struct.pack("<H", raw) + b"\x00" * 6)