from dataclasses import dataclass
from abc import ABC, abstractmethod

from sparkmax_definitions import *


class CANNode(ABC):
    def __init__(self, can_id: int):
        self.can_id = can_id
        self.accepts_velocity = False
        self.accepts_duty = False

    @abstractmethod
    def write(self, serial): ...

class DutyCycleDevice(ABC):
    def __init__(self):
        self.accepts_velocity = True

    @abstractmethod
    def set_duty(self, duty: float): ...

class VelocityDevice(ABC):
    def __init__(self):
        self.accepts_duty = True

    @abstractmethod
    def set_velocity(self, rpm: float): ...


class Motor(CANNode, DutyCycleDevice, VelocityDevice):
    def __init__(self, can_id: int, max_rpm: float = 5700.0):
        super().__init__(can_id)
        self.max_rpm = max_rpm
        self.commanded_rpm = 0.0
        self.measured_rpm = 0.0
        self.position_rotations = 0.0

    def set_duty(self, duty):
        self.commanded_rpm = duty * self.max_rpm

    def set_velocity(self, rpm):
        self.commanded_rpm = rpm

    def update(self, time_change: float):
        """
        Updates to move towards commanded value.
        :param time_change: change in time since last update.
        """
        cmd, meas, pos = self.commanded_rpm, self.measured_rpm, self.position_rotations
        # Alpha: how far to step toward target this tick.
        # Derived from e^(-dt/tau) where tau=0.15s is the time constant.
        # At tau, the response reaches ~63% of the commanded value.
        alpha = 1.0 - 2.718 ** (-time_change / 0.15)
        meas += alpha * (cmd - meas)
        pos  += meas / 60.0 * time_change   # RPM -> rotations/sec -> rotations
        self.measured_rpm, self.position_rotations = meas, pos

    def write(self, serial):
        can_id = self.can_id
        applied = max(-1.0, min(1.0, self.measured_rpm / self.max_rpm))
        serial.write(create_applied_output_status(can_id, applied))
        serial.write(create_velocity_status(can_id, self.measured_rpm))
        serial.write(create_position_status(can_id, self.position_rotations))

    def __repr__(self):
        return f"(ID={self.can_id} rpm={self.measured_rpm:.0f}/{self.max_rpm:.0f})"

@dataclass
class Actuator(CANNode, DutyCycleDevice):
    def __init__(self, can_id: int):
        super().__init__(can_id)
        self.commanded_duty = 0.0
        self.position = 0.5

    def set_duty(self, duty):
        self.commanded_duty = max(-1.0, min(1.0, duty))

    def update(self, time_change):
        # Full travel (0->1) takes 2 seconds at full duty (speed=0.5 pos/sec)
        self.position = max(0.0, min(1.0, self.position + self.commanded_duty * 0.5 * time_change))

    def write(self, serial):
        can_id = self.can_id
        # Map actuator position (0-1) back to voltage range
        voltage = 0.279 + self.position * (1.85 - 0.279)
        serial.write(create_applied_output_status(can_id, self.commanded_duty))
        serial.write(create_analog_status(can_id, voltage))

    def __repr__(self):
        return f"(ID={self.can_id} {self.commanded_duty:.3f}/{self.position:.3f})"
