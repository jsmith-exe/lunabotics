import os, time, serial, logging, threading


from .can_helpers import create_packed_id, HEARTBEAT_ID
from .sparkmax_helpers import *
from .waveshare_helpers import read_waveshare_frame_from_serial
from .classes import Motor, Actuator


def create_logger():
    logger = logging.getLogger("CANSim")
    console_handler = logging.StreamHandler()
    formatter = logging.Formatter(
        "{asctime} {levelname}: {message}",
        style="{",
        datefmt="%M:%S",
    )
    console_handler.setFormatter(formatter)

    logger.addHandler(console_handler)
    logger.setLevel(logging.INFO)
    return logger


class CANSim:
    """
    Accepts a CAN network described as a dict, e.g.:
    {1: Motor(1), 5: Actuator(5)}

    When the start method is called, it will connect to provided serial port and start simulating the CAN network.
    Motors will accept duty and velocity commands.
    Linear actuators will accept duty commands.
    Transmit functionality delegated to motor and actuator classes.
    """

    def __init__(self, can_id_to_state_mapping: dict[int, Motor | Actuator], port, baudrate):
        self.logger = create_logger()

        self.can_id_to_state_mapping = can_id_to_state_mapping
        # Generate helper data structures
        self.can_ids = can_id_to_state_mapping.keys()
        self.can_devices = can_id_to_state_mapping.values()
        self.velocity_can_ids = [can_id for can_id, device in self.can_id_to_state_mapping.items() if isinstance(device, Motor)]
        # Map raw binary representing CAN IDs for each device's setpoint commands - create for both duty cycle and velocity.
        # The raw CAN ID is different from the CAN ID - raw CAN ID refers the binary encoded ID directly from the serial port.
        # Example of duty mapping: {33882241: 1, 33882242: 2, 33882243: 3}
        self.raw_duty_can_id_to_state_mapping = {create_packed_id(0x02, can_id): self.can_id_to_state_mapping[can_id] for can_id in self.can_ids}
        self.raw_vel_can_id_to_state_mapping = {create_packed_id(0x12, can_id): self.can_id_to_state_mapping[can_id] for can_id in self.velocity_can_ids}

        self.serial = serial.Serial(timeout=0.005) # Block for 5ms if no data - most stable at this
        self.serial.port = port
        self.serial.baudrate = baudrate

        self.serial_lock = threading.Lock()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)

    def start(self):
        """
        Opens the configured serial port and starts the simulation loop.
        """
        self.logger.info(f"Waiting for {self.serial.port}")
        while not os.path.exists(self.serial.port):
            time.sleep(0.1)
        self.serial.open()
        self.logger.info(f"Opened {self.serial.port} with {self.serial.baudrate}")
        self.rx_thread.start()
        self.tx_thread.start()

    def _rx_loop(self):
        while True:
            # Read frames
            with self.serial_lock:
                frame = read_waveshare_frame_from_serial(self.serial)
            if frame: self._parse_frame(frame)

    def _tx_loop(self):
        # Keep track of next write and log times because they're periodic
        next_write = time.monotonic()
        next_log = time.monotonic()
        write_period = 1.0 / 50 # 50 Hz
        log_period = 1.0
        last_update = time.monotonic() # Keep track of last update, because updates are continuously happening

        while True:
            # Update motors
            now = time.monotonic()
            time_change = min(now - last_update, 0.1)
            last_update = now
            for device in self.can_devices:
                device.update(time_change)

            # Transmit status TODO fix minor lag during writing
            if now >= next_write:
                next_write = now + write_period
                with self.serial_lock:
                    for device in self.can_devices:
                        device.write(self.serial)

            # Log results
            if now >= next_log:
                next_log = now + log_period
                info = "  ".join(f"{device}" for device in self.can_devices)
                self.logger.info(f"[{now:.1f}] {info}")

    def _parse_frame(self, frame):
        raw_can_id, data = frame
        sufficient_data_to_decode = len(data) >= 4

        if raw_can_id == HEARTBEAT_ID:
            pass

        # Handle duty commands
        elif raw_can_id in self.raw_duty_can_id_to_state_mapping and sufficient_data_to_decode:
            device = self.raw_duty_can_id_to_state_mapping[raw_can_id]
            duty = struct.unpack_from("<f", data)[0]   # float32 LE
            device.set_duty(duty)

        # Handle velocity commands
        elif raw_can_id in self.raw_vel_can_id_to_state_mapping:
            device = self.raw_vel_can_id_to_state_mapping[raw_can_id]
            commanded_vel = struct.unpack_from("<f", data)[0] # RPM as float32 LE
            device.set_velocity(commanded_vel)
