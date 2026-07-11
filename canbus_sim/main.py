"""
FoilBus is a SPARK MAX simulator - a stand in for the motor and actuator setup.
Advantages:
 - No wiring
 - No access to physical rover needed

With socat installed, run:
socat PTY,link=/dev/ttyUSB0,rawer PTY,link=/tmp/fake_can_rx,rawer
This creates a fake USB /dev/ttyUSB0, which talks to /tmp/fake_can_rx, which will be written to by this script.
The diffdrive canbus system will connect to the fake USB.
"""
import argparse, os, time, serial, logging, dataclasses
from sparkmax_definitions import *
logging.basicConfig()

# Device IDs present on this simulated bus, matching the robot's wiring.
ALL_IDS      = [1, 2, 3, 4, 5, 6, 7]   # 1-4: wheels, 5-6: actuators, 7: drum
ACTUATOR_IDS = {5, 6}                    # these also send analog voltage (Status 3)

# TODO inheritance?
@dataclasses.dataclass
class Motor:
    can_id: int
    commanded_rpm: float = 0.0
    measured_rpm: float = 0.0
    position_rotations: float = 0.0

    def update(self, dt: float):
        """
        Updates to move towards commanded value.
        :param dt: change in time since last update.
        """
        cmd, meas, pos = self.commanded_rpm, self.measured_rpm, self.position_rotations
        # Alpha: how far to step toward target this tick.
        # Derived from e^(-dt/tau) where tau=0.15s is the time constant.
        # At tau, the response reaches ~63% of the commanded value.
        alpha = 1.0 - 2.718 ** (-dt / 0.15)
        meas += alpha * (cmd - meas)
        pos  += meas / 60.0 * dt   # RPM -> rotations/sec -> rotations
        self.measured_rpm, self.position_rotations = meas, pos

@dataclasses.dataclass
class Actuator:
    can_id: int
    commanded_duty: float = 0.0
    position: float = 0.5

    def update(self, dt):
        # Full travel (0->1) takes 2 seconds at full duty (speed=0.5 pos/sec)
        self.position = max(0.0, min(1.0, self.position + self.commanded_duty * 0.5 * dt))

class CANBusSim:
    def __init__(self, can_id_to_state_mapping: dict[int, Motor | Actuator], port, baudrate):
        self.can_id_to_state_mapping = can_id_to_state_mapping
        self.can_ids = can_id_to_state_mapping.keys()
        # Map raw binary representing CAN IDs for each device's setpoint commands - create for both duty cycle and velocity.
        # Example of duty mapping: {33882241: 1, 33882242: 2, 33882243: 3}
        self.raw_duty_can_id_to_state_mapping = {create_packed_id(0x02, can_id): self.can_id_to_state_mapping[can_id] for can_id in self.can_ids}
        self.raw_vel_can_id_to_state_mapping = {create_packed_id(0x12, can_id): self.can_id_to_state_mapping[can_id] for can_id in self.can_ids}

        self.port = port
        self.baudrate = baudrate
        self.serial = serial.Serial(timeout=0.005)
        self.serial.port = port
        self.serial.baudrate = baudrate

        self.logger = logging.getLogger("CANBusSim")
        self.logger.setLevel(logging.INFO)

    def start(self):
        self.logger.info(f"Waiting for {self.port} ...")
        while not os.path.exists(self.port):
            time.sleep(0.1)
        self.serial.open()
        self.logger.info(f"Opened {self.port} with {self.baudrate}")

    def loop(self):
        next_tx   = time.monotonic()
        next_stat = time.monotonic()
        TX_PERIOD = 1.0 / 50    # 50 Hz — fast enough to keep plugin feedback stable

        while True:
            # --- RX: decode one incoming frame per loop iteration ---
            # The serial timeout (0.005s) means this blocks for at most 5ms
            # when no data is available, which limits the loop rate under low load.
            frame = read_waveshare_frame_from_serial(self.serial)
            if frame:
                self.parse_frame(frame)

            # --- Dynamics: first-order lag toward commanded value ---
            # dt is capped at 0.1s to prevent large jumps after pauses.
            now = time.monotonic()
            dt  = min(now - next_tx + TX_PERIOD, 0.1)
            for device in self.can_id_to_state_mapping.values():
                device.update(dt)

            # --- TX: broadcast status frames at 50 Hz ---
            if now >= next_tx:
                next_tx = now + TX_PERIOD
                for can_id, device in self.can_id_to_state_mapping.items():
                    # cmd, meas, pos =
                    if isinstance(device, Actuator):
                        # Map actuator position (0-1) back to voltage range
                        voltage = 0.279 + device.position * (1.85 - 0.279)
                        self.serial.write(create_position_status(can_id, device.position))
                        self.serial.write(create_analog_status(can_id, voltage))
                    elif isinstance(device, Motor):
                        applied = max(-1.0, min(1.0, device.measured_rpm / 5700.0))
                        self.serial.write(create_applied_output_status(can_id, applied))
                        self.serial.write(create_velocity_status(can_id, device.measured_rpm))
                    else:
                        raise ValueError(f"Invalid device class: {type(device)}")

            # --- Stats: print motor state every 5 seconds ---
            if now >= next_stat:
                next_stat = now + 5.0
                info = "  ".join(
                    (f"id={can_id} rpm={device.measured_rpm:+.0f}" if isinstance(device, Motor) else "") +
                    (f" apos={device.position:.2f}" if isinstance(device, Actuator) else "")
                    for can_id, device in self.can_id_to_state_mapping.items()
                )
                print(f"[{now:.1f}] {info}")

    def parse_frame(self, frame):
        raw_can_id, data = frame
        sufficient_data_to_decode = len(data) >= 4

        if raw_can_id == HEARTBEAT_ID:
            pass

        elif raw_can_id in self.raw_duty_can_id_to_state_mapping and sufficient_data_to_decode:
            device = self.raw_duty_can_id_to_state_mapping[raw_can_id]
            duty = struct.unpack_from("<f", data)[0]   # float32 LE
            if isinstance(device, Actuator):
                device.commanded_duty = max(-1.0, min(1.0, duty))
            elif isinstance(device, Motor):
                # Approximate RPM from duty: NEO free-speed ~5700 RPM at full duty
                device.commanded_rpm = duty * 5700.0
            else:
                raise ValueError(f"Invalid device class: {type(device)}")

        elif raw_can_id in self.raw_vel_can_id_to_state_mapping:
            device = self.raw_vel_can_id_to_state_mapping[raw_can_id]
            if isinstance(device, Actuator):
                raise ValueError(f"Invalid device class: {type(device)}")
            elif isinstance(device, Motor):
                # Approximate RPM from duty: NEO free-speed ~5700 RPM at full duty
                device.commanded_rpm = struct.unpack_from("<f", data)[0] # RPM as float32 LE
            else:
                raise ValueError(f"Invalid device class: {type(device)}")


def main():
    ap = argparse.ArgumentParser(description="SPARK MAX simulator for diffdrive_canbus")
    ap.add_argument("--port", default="/tmp/fake_can_rx", help="Serial port (socat PTY end)")
    ap.add_argument("--baud", type=int, default=2000000, help="Must match serial_baud_rate in ros2_control params")
    args = ap.parse_args()

    can_setup = {
        1: Motor(1),
        2: Motor(2),
        3: Motor(3),
        4: Motor(4),
        5: Actuator(5),
        6: Actuator(6),
        7: Motor(7),
    }

    sim = CANBusSim(can_setup, args.port, args.baud)
    sim.start()
    sim.loop()
    # ser = sim.serial
    #
    #
    #
    # ## Left off here - what are DUTY_IDS and VEL_IDS for?
    # # TODO switch state and act to use can_setup.
    # # TODO move DUTY_IDS and VEL_IDS if needed
    #
    # # Motor state per device: [commanded_rpm, measured_rpm, position_rotations]
    # # Wheels and drum use RPM; actuators use duty cycle separately.
    # state__ = {i: [0.0, 0.0, 0.0] for i in ALL_IDS}
    #
    # # Actuator state per device: [commanded_duty (-1 to 1), position (0 to 1)]
    # # Position 0.0 = fully retracted, 1.0 = fully extended.
    # act = {i: [0.0, 0.5] for i in ACTUATOR_IDS}
    #
    # # Pre-compute expected CAN IDs for each device's setpoint commands.
    # # The plugin uses packed api_ids: 0x02 for duty cycle, 0x12 for velocity.
    # DUTY_IDS = {create_packed_id(0x02, i): i for i in ALL_IDS}
    # VEL_IDS  = {create_packed_id(0x12, i): i for i in ALL_IDS}

if __name__ == "__main__":
    main()
