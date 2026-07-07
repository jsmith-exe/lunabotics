#!/usr/bin/env python3
"""
SPARK MAX simulator stub for the diffdrive_canbus ROS 2 hardware plugin.

Sits on one end of a socat virtual serial pair and speaks the Waveshare
USB-CAN-A binary protocol. Decodes incoming config packets, heartbeats, and
setpoint commands, then periodically broadcasts plausible SPARK MAX periodic
status frames back so the plugin's feedback path stays happy.

Usage
-----
1. Create a virtual serial pair:

       socat PTY,link=/tmp/fake_usb,rawer PTY,link=/tmp/fake_can_rx,rawer

2. Run this stub on the simulator end:

       python3 sparkmax_sim.py --port /tmp/fake_can_rx

3. Point your plugin at the other end in your ros2_control hardware params:

       <param name="serial_device">/tmp/fake_usb</param>

Motor dynamics
--------------
Each simulated SPARK MAX tracks a first-order velocity response: the measured
RPM moves toward the commanded RPM with a configurable time constant
(default 0.15 s). Position is integrated from velocity each tick. The analog
position feedback for the two linear actuators is synthesised from their
duty-cycle commands and reported in Periodic Status 3.

FRC CAN ID layout (29-bit extended)
------------------------------------
  bits 28-24  device type  (5 bits)  0x02 = motor controller
  bits 23-16  manufacturer (8 bits)  0x05 = REV
  bits 15-10  api class    (6 bits)
  bits  9- 6  api index    (4 bits)
  bits  5- 0  device id    (6 bits)

Waveshare USB-CAN-A frame format
---------------------------------
  TX (host -> adapter):
    [0xAA] [type] [id_lo id_hi (std) | id_0..id_3 (ext)] [data 0..dlc-1] [0x55]

  type byte:
    bits 7-6  must be 0b11
    bit  5    1 = extended 29-bit ID
    bit  4    1 = RTR / remote frame
    bits 3-0  DLC (0-8)

  Config packet (20 bytes):
    [0xAA] [0x55] [0x12] [baud_code] [filter_type]
    [filter_id 4 bytes LE] [block_id 4 bytes LE]
    [mode] [disable_retransmit] [0x00]*4 [checksum]
"""

import argparse
import logging
import struct
import threading
import time
import serial

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("sparkmax_sim")

# ---------------------------------------------------------------------------
# FRC / SPARK MAX constants
# ---------------------------------------------------------------------------
DEVICE_TYPE_MOTOR_CONTROLLER = 0x02
MANUFACTURER_REV              = 0x05

# API classes
API_CLASS_DUTY_CYCLE    = 0x00
API_CLASS_VELOCITY      = 0x01
API_CLASS_STATUS_FW24   = 0x06   # Periodic status frames, firmware 24.x
API_CLASS_NON_RIO       = 0x16   # Non-RIO heartbeat

# API indices
API_INDEX_DUTY_CYCLE_SET  = 0x00
API_INDEX_VELOCITY_SET    = 0x02
API_INDEX_STATUS_0        = 0x00
API_INDEX_STATUS_1        = 0x01
API_INDEX_STATUS_2        = 0x02
API_INDEX_NON_RIO_HB      = 0x00

# Status 3 base ID (analog sensor / linear actuator feedback)
STATUS3_BASE_ID = 0x020518C0

# Heartbeat uses a broadcast device ID
HEARTBEAT_DEVICE_ID = 0x00

# The CAN_comms.cpp SPARKMAX_API_DUTY_CYCLE_SET / SPARKMAX_API_VELOCITY_SET
# are packed as an 8-bit api_id where upper nibble = api_class, lower = api_index.
# 0x02 -> class=0, index=2   (duty cycle set)
# 0x12 -> class=1, index=2   (velocity set)
PACKED_API_DUTY_CYCLE = 0x02
PACKED_API_VELOCITY   = 0x12

# Simulated device IDs present on the bus
WHEEL_IDS    = [1, 2, 3, 4]   # front-left, front-right, rear-left, rear-right
ACTUATOR_IDS = [5, 6]          # left actuator, right actuator
DRUM_ID      = 7
ALL_IDS      = WHEEL_IDS + ACTUATOR_IDS + [DRUM_ID]

# Motor dynamics
VELOCITY_TIME_CONSTANT = 0.15   # seconds - first-order lag
STATUS_BROADCAST_HZ    = 50     # how often to push status frames

# Linear actuator analog voltage range (matching plugin defaults)
ACTUATOR_VOLTAGE_MIN = 0.279
ACTUATOR_VOLTAGE_MAX = 1.850

# ---------------------------------------------------------------------------
# FRC CAN ID helpers
# ---------------------------------------------------------------------------

def make_frc_id(device_type, manufacturer, api_class, api_index, device_id):
    return (
            ((device_type  & 0x1F) << 24) |
            ((manufacturer & 0xFF) << 16) |
            ((api_class    & 0x3F) << 10) |
            ((api_index    & 0x0F) <<  6) |
            ((device_id    & 0x3F) <<  0)
    )


def parse_frc_id(can_id):
    return {
        "device_type":  (can_id >> 24) & 0x1F,
        "manufacturer": (can_id >> 16) & 0xFF,
        "api_class":    (can_id >> 10) & 0x3F,
        "api_index":    (can_id >>  6) & 0x0F,
        "device_id":     can_id        & 0x3F,
    }


def make_sparkmax_id(api_class, api_index, device_id):
    return make_frc_id(
        DEVICE_TYPE_MOTOR_CONTROLLER,
        MANUFACTURER_REV,
        api_class,
        api_index,
        device_id,
    )


def make_sparkmax_id_from_packed(api_id, device_id):
    """Split 8-bit packed api_id (upper nibble = class, lower = index)."""
    api_class = (api_id >> 4) & 0x3F
    api_index =  api_id       & 0x0F
    return make_sparkmax_id(api_class, api_index, device_id)


# Pre-compute command CAN IDs for fast lookup
DUTY_CMD_IDS     = {did: make_sparkmax_id_from_packed(PACKED_API_DUTY_CYCLE, did) for did in ALL_IDS}
VELOCITY_CMD_IDS = {did: make_sparkmax_id_from_packed(PACKED_API_VELOCITY,   did) for did in ALL_IDS}

# Heartbeat ID (non-RIO broadcast)
HEARTBEAT_ID = make_sparkmax_id(API_CLASS_NON_RIO, API_INDEX_NON_RIO_HB, HEARTBEAT_DEVICE_ID)

# ---------------------------------------------------------------------------
# Waveshare frame codec
# ---------------------------------------------------------------------------

def encode_waveshare_frame(can_id, data, extended=True):
    """Encode a CAN frame in Waveshare USB-CAN-A format."""
    dlc = len(data)
    assert dlc <= 8

    type_byte = 0xC0
    if extended:
        type_byte |= (1 << 5)
    type_byte |= (dlc & 0x0F)

    packet = bytearray()
    packet.append(0xAA)
    packet.append(type_byte)

    if extended:
        packet.append((can_id >>  0) & 0xFF)
        packet.append((can_id >>  8) & 0xFF)
        packet.append((can_id >> 16) & 0xFF)
        packet.append((can_id >> 24) & 0x1F)
    else:
        packet.append((can_id >> 0) & 0xFF)
        packet.append((can_id >> 8) & 0x07)

    packet.extend(data)
    packet.append(0x55)
    return bytes(packet)


class WaveshareReader:
    """
    Stateful incremental parser for the Waveshare USB-CAN-A binary stream.

    Call feed(byte) with each byte read from the serial port. When a complete
    frame or config packet is assembled it is returned as a dict.
    """

    def __init__(self):
        self._buf = bytearray()
        self._in_frame = False

    def feed(self, byte):
        """
        Feed one byte. Returns a parsed object or None.

        Returned dict keys:
          kind      "frame" | "config" | "unknown"
          For "frame":
            extended, remote, dlc, can_id, data (bytes)
          For "config":
            baud_code, filter_type, filter_id, block_id, mode,
            disable_retransmit
        """
        b = byte if isinstance(byte, int) else byte[0]

        if not self._in_frame:
            if b == 0xAA:
                self._buf = bytearray([b])
                self._in_frame = True
            return None

        self._buf.append(b)

        # Need at least 2 bytes to decide what kind of packet this is.
        if len(self._buf) < 2:
            return None

        second = self._buf[1]

        # Config packet: second byte is 0x55, third byte is 0x12
        if second == 0x55:
            if len(self._buf) < 3:
                return None
            if self._buf[2] != 0x12:
                # Not a config packet - discard and resync
                self._in_frame = False
                return None
            if len(self._buf) < 20:
                return None
            # Full config packet received
            pkt = bytes(self._buf)
            self._in_frame = False
            return self._parse_config(pkt)

        # CAN frame: second byte has top bits 0b11xxxxxx
        if (second & 0xC0) != 0xC0:
            self._in_frame = False
            return None

        extended = bool((second >> 5) & 0x01)
        remote   = bool((second >> 4) & 0x01)
        dlc      = second & 0x0F

        if dlc > 8:
            self._in_frame = False
            return None

        id_len   = 4 if extended else 2
        data_len = 0 if remote else dlc
        total    = 1 + 1 + id_len + data_len + 1   # AA + type + id + data + 55

        if len(self._buf) < total:
            return None

        if self._buf[total - 1] != 0x55:
            self._in_frame = False
            return None

        raw = bytes(self._buf[:total])
        self._in_frame = False
        return self._parse_frame(raw, extended, remote, dlc, id_len)

    @staticmethod
    def _parse_frame(raw, extended, remote, dlc, id_len):
        if extended:
            can_id = (
                    (raw[2] <<  0) |
                    (raw[3] <<  8) |
                    (raw[4] << 16) |
                    ((raw[5] & 0x1F) << 24)
            )
        else:
            can_id = raw[2] | ((raw[3] & 0x07) << 8)

        data_offset = 2 + id_len
        data = raw[data_offset:data_offset + (0 if remote else dlc)]

        return {
            "kind":     "frame",
            "extended": extended,
            "remote":   remote,
            "dlc":      dlc,
            "can_id":   can_id,
            "data":     bytes(data),
        }

    @staticmethod
    def _parse_config(pkt):
        baud_code          = pkt[3]
        filter_type        = pkt[4]
        filter_id          = struct.unpack_from("<I", pkt, 5)[0]
        block_id           = struct.unpack_from("<I", pkt, 9)[0]
        mode               = pkt[13]
        disable_retransmit = pkt[14]
        return {
            "kind":               "config",
            "baud_code":          baud_code,
            "filter_type":        filter_type,
            "filter_id":          filter_id,
            "block_id":           block_id,
            "mode":               mode,
            "disable_retransmit": disable_retransmit,
        }

# ---------------------------------------------------------------------------
# Motor / actuator state
# ---------------------------------------------------------------------------

class MotorState:
    """First-order motor dynamics for one simulated SPARK MAX."""

    def __init__(self, device_id, is_actuator=False):
        self.device_id   = device_id
        self.is_actuator = is_actuator

        # Velocity control
        self.commanded_rpm  = 0.0
        self.measured_rpm   = 0.0
        self.position_rot   = 0.0   # rotations

        # Duty cycle (actuators only)
        self.commanded_duty = 0.0

        # Analog position for actuators: starts mid-range
        self.actuator_position = 0.5   # 0.0 = retracted, 1.0 = extended
        self.actuator_voltage  = (
                ACTUATOR_VOLTAGE_MIN +
                0.5 * (ACTUATOR_VOLTAGE_MAX - ACTUATOR_VOLTAGE_MIN)
        )

        self._last_update = time.monotonic()

    def update(self):
        now = time.monotonic()
        dt  = now - self._last_update
        self._last_update = now

        if self.is_actuator:
            # Move actuator position based on duty command
            # Duty +1.0 = full extend, -1.0 = full retract
            # Full travel (0 -> 1) takes roughly 2 seconds at full duty
            speed = 0.5   # positions per second at full duty
            self.actuator_position = max(0.0, min(1.0,
                                                  self.actuator_position + self.commanded_duty * speed * dt
                                                  ))
            self.actuator_voltage = (
                    ACTUATOR_VOLTAGE_MIN +
                    self.actuator_position * (ACTUATOR_VOLTAGE_MAX - ACTUATOR_VOLTAGE_MIN)
            )
            # Also track RPM = 0 for actuators (not meaningful but keeps
            # the status frames consistent)
            self.measured_rpm = 0.0
        else:
            # First-order lag toward commanded RPM
            alpha = 1.0 - pow(2.718281828, -dt / VELOCITY_TIME_CONSTANT)
            self.measured_rpm += alpha * (self.commanded_rpm - self.measured_rpm)
            # Integrate position
            self.position_rot += self.measured_rpm / 60.0 * dt

    def applied_output(self):
        """Approximate applied output in [-1, 1] from RPM or duty."""
        if self.is_actuator:
            return self.commanded_duty
        # Rough estimate: assume max ~5700 RPM (NEO free speed)
        return max(-1.0, min(1.0, self.measured_rpm / 5700.0))


# ---------------------------------------------------------------------------
# Status frame builders
# ---------------------------------------------------------------------------

def build_status0(device_id, applied_output):
    """
    Periodic Status 0 (firmware 24):
      CAN ID: 0x2051800 + device_id
      bytes 0-1: applied output as int16 scaled by 32767
    """
    can_id   = make_sparkmax_id(API_CLASS_STATUS_FW24, API_INDEX_STATUS_0, device_id)
    raw_out  = max(-32767, min(32767, int(applied_output * 32767)))
    data     = struct.pack("<h", raw_out) + b"\x00" * 6
    return encode_waveshare_frame(can_id, data)


def build_status1(device_id, velocity_rpm):
    """
    Periodic Status 1 (firmware 24):
      CAN ID: 0x2051840 + device_id
      bytes 0-3: velocity RPM as float32 LE
    """
    can_id = make_sparkmax_id(API_CLASS_STATUS_FW24, API_INDEX_STATUS_1, device_id)
    data   = struct.pack("<f", velocity_rpm) + b"\x00" * 4
    return encode_waveshare_frame(can_id, data)


def build_status2(device_id, position_rotations):
    """
    Periodic Status 2 (firmware 24):
      CAN ID: 0x2051880 + device_id
      bytes 0-3: encoder position rotations as float32 LE
    """
    can_id = make_sparkmax_id(API_CLASS_STATUS_FW24, API_INDEX_STATUS_2, device_id)
    data   = struct.pack("<f", position_rotations) + b"\x00" * 4
    return encode_waveshare_frame(can_id, data)


def build_status3_actuator(device_id, analog_voltage):
    """
    Periodic Status 3 (analog sensor):
      CAN ID: STATUS3_BASE_ID + device_id
      bytes 0-1: adcVoltage in 2q8 fixed point (voltage * 256), uint16 LE
    """
    can_id    = STATUS3_BASE_ID + device_id
    raw_adc   = max(0, min(0xFFFF, int(analog_voltage * 256)))
    data      = struct.pack("<H", raw_adc) + b"\x00" * 6
    return encode_waveshare_frame(can_id, data)


# ---------------------------------------------------------------------------
# Main simulator class
# ---------------------------------------------------------------------------

class SparkMaxSim:

    def __init__(self, port, baud=2000000):
        self.port  = port
        self.baud  = baud
        self._ser  = None
        self._lock = threading.Lock()

        # One MotorState per device ID
        self._motors = {
            did: MotorState(did, is_actuator=(did in ACTUATOR_IDS))
            for did in ALL_IDS
        }

        # Build reverse lookup: CAN ID -> (device_id, command_type)
        self._cmd_lookup = {}
        for did in ALL_IDS:
            self._cmd_lookup[DUTY_CMD_IDS[did]]     = (did, "duty")
            self._cmd_lookup[VELOCITY_CMD_IDS[did]] = (did, "velocity")

        self._heartbeat_count = 0
        self._frame_rx_count  = 0
        self._frame_tx_count  = 0
        self._running         = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def run(self):
        log.info("Opening serial port %s @ %d baud", self.port, self.baud)

        self._ser = serial.Serial(
            self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.005,     # short non-blocking read timeout
        )

        self._running = True

        rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        tx_thread = threading.Thread(target=self._tx_loop, daemon=True)

        rx_thread.start()
        tx_thread.start()

        log.info("Simulator running. Device IDs: %s", ALL_IDS)
        log.info("Press Ctrl+C to stop.")

        try:
            while True:
                time.sleep(1.0)
                self._print_stats()
        except KeyboardInterrupt:
            log.info("Stopping simulator.")
        finally:
            self._running = False
            rx_thread.join(timeout=2.0)
            tx_thread.join(timeout=2.0)
            self._ser.close()

    # ------------------------------------------------------------------
    # RX: decode incoming frames from the plugin
    # ------------------------------------------------------------------

    def _rx_loop(self):
        reader = WaveshareReader()

        while self._running:
            try:
                raw = self._ser.read(1)
            except serial.SerialException as exc:
                log.error("Serial read error: %s", exc)
                break

            if not raw:
                continue

            result = reader.feed(raw[0])
            if result is None:
                continue

            if result["kind"] == "config":
                log.info(
                    "Config packet: baud_code=0x%02X filter_type=%d mode=%d",
                    result["baud_code"],
                    result["filter_type"],
                    result["mode"],
                )
                continue

            if result["kind"] != "frame":
                continue

            self._frame_rx_count += 1
            self._handle_rx_frame(result)

    def _handle_rx_frame(self, frame):
        can_id = frame["can_id"]

        # Heartbeat
        if can_id == HEARTBEAT_ID:
            self._heartbeat_count += 1
            log.debug("Heartbeat #%d", self._heartbeat_count)
            return

        # Setpoint command
        if can_id in self._cmd_lookup:
            device_id, cmd_type = self._cmd_lookup[can_id]
            motor = self._motors.get(device_id)
            if motor is None:
                return

            if len(frame["data"]) < 4:
                return

            value = struct.unpack_from("<f", frame["data"], 0)[0]

            with self._lock:
                if cmd_type == "duty":
                    motor.commanded_duty = max(-1.0, min(1.0, value))
                    # For wheels driven in duty mode, approximate RPM
                    if not motor.is_actuator:
                        motor.commanded_rpm = value * 5700.0
                    log.debug(
                        "DUTY  id=%d  duty=%.4f", device_id, motor.commanded_duty
                    )
                elif cmd_type == "velocity":
                    motor.commanded_rpm = value
                    log.debug(
                        "VEL   id=%d  rpm=%.1f", device_id, motor.commanded_rpm
                    )
            return

        # Everything else - log at debug level only
        fields = parse_frc_id(can_id)
        log.debug(
            "RX unknown: can_id=0x%08X dev_type=%d mfr=%d api_class=%d api_index=%d dev_id=%d",
            can_id,
            fields["device_type"],
            fields["manufacturer"],
            fields["api_class"],
            fields["api_index"],
            fields["device_id"],
        )

    # ------------------------------------------------------------------
    # TX: broadcast status frames at STATUS_BROADCAST_HZ
    # ------------------------------------------------------------------

    def _tx_loop(self):
        period = 1.0 / STATUS_BROADCAST_HZ

        while self._running:
            start = time.monotonic()

            with self._lock:
                for motor in self._motors.values():
                    motor.update()

            self._send_all_status_frames()

            elapsed = time.monotonic() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _send_all_status_frames(self):
        frames = []

        with self._lock:
            for did, motor in self._motors.items():
                ao = motor.applied_output()

                frames.append(build_status0(did, ao))
                frames.append(build_status1(did, motor.measured_rpm))
                frames.append(build_status2(did, motor.position_rot))

                if motor.is_actuator:
                    frames.append(
                        build_status3_actuator(did, motor.actuator_voltage)
                    )

        for frame_bytes in frames:
            try:
                self._ser.write(frame_bytes)
                self._frame_tx_count += 1
            except serial.SerialException as exc:
                log.error("Serial write error: %s", exc)
                return

    # ------------------------------------------------------------------
    # Stats
    # ------------------------------------------------------------------

    def _print_stats(self):
        with self._lock:
            motor_info = [
                f"id={m.device_id} rpm={m.measured_rpm:+.0f}"
                + (f" pos={m.actuator_position:.2f}" if m.is_actuator else "")
                for m in self._motors.values()
            ]

        log.info(
            "rx=%d tx=%d heartbeats=%d | %s",
            self._frame_rx_count,
            self._frame_tx_count,
            self._heartbeat_count,
            "  ".join(motor_info),
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPARK MAX simulator stub for diffdrive_canbus"
    )
    parser.add_argument(
        "--port",
        default="/tmp/fake_can_rx",
        help="Serial port to listen on (default: /tmp/fake_can_rx)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=2000000,
        help="Serial baud rate, must match serial_baud_rate in ros2_control params (default: 2000000)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable DEBUG level logging",
    )
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    sim = SparkMaxSim(port=args.port, baud=args.baud)
    sim.run()


if __name__ == "__main__":
    main()