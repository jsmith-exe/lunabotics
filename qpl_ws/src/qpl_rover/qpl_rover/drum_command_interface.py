"""
Prioritises subscriptions from teleop and autonomy control, automatically zeroes commands on absence of data, and
 formats data for suitable publishing.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from enum import Enum, auto
import time

class ControlSource(Enum):
    NONE = auto()
    AUTONOMY = auto()
    TELEOP = auto()


class DrumInterface(Node):
    def __init__(self):
        super().__init__("drum_command_interface")
        self.lift_autonomy_sub = self.create_subscription(
            Float64, "/drum_lift_control/autonomy", self.lift_autonomy_cb, 10
        )
        self.spin_autonomy_sub = self.create_subscription(
            Float64, "/drum_spin_control/autonomy", self.spin_autonomy_cb, 10
        )
        self.lift_teleop_sub = self.create_subscription(
            Float64, "/drum_lift_control/teleop", self.lift_teleop_cb, 10
        )
        self.spin_teleop_sub = self.create_subscription(
            Float64, "/drum_spin_control/teleop", self.spin_teleop_cb, 10
        )

        self.drum_lift_pub = self.create_publisher(
            Float64MultiArray, "/drum_lift_cont/commands", 10
        )
        self.drum_spin_pub = self.create_publisher(
            Float64MultiArray, "/drum_cont/commands", 10
        )

        # Automatically zero commands if nothing received.
        self.zero_timer_check_rate = 2 # seconds
        self.zero_timer_threshold_seconds = 1.5 # How long there must be no commands before zeroing
        self.zero_timer = self.create_timer(1/self.zero_timer_check_rate, self.send_zero_if_no_recent_messages)
        self.last_message_received_time = 0

        self.prioritised_source = ControlSource.TELEOP
        self.last_priority_message_time = 0
        self.priority_timeout_seconds = 5 # How long to wait before accepting deprioritised messages
        self.get_logger().info("Drum command interface node initialised")

    def set_last_message_time(self):
        self.last_message_received_time = time.time()

    def lift_autonomy_cb(self, msg: Float64):
        if self.skip_if_deprioritised(ControlSource.AUTONOMY): return
        msg.data = normalise_for_actuator_convention(msg.data)
        self.set_drum_lift_rate(msg.data)

    def spin_autonomy_cb(self, msg: Float64):
        if self.skip_if_deprioritised(ControlSource.AUTONOMY): return
        self.set_drum_spin_rate(msg.data)

    def lift_teleop_cb(self, msg: Float64):
        if self.skip_if_deprioritised(ControlSource.TELEOP): return
        msg.data = normalise_for_actuator_convention(msg.data)
        self.set_drum_lift_rate(msg.data)

    def spin_teleop_cb(self, msg: Float64):
        if self.skip_if_deprioritised(ControlSource.TELEOP): return
        self.set_drum_spin_rate(msg.data)

    def set_drum_lift_rate(self, lift: float):
        """ Sets the drum lift rate.
        :param lift: The desired lift rate, between 0 and 1 inclusive.
        """
        float_array = Float64MultiArray()
        float_array.data = [lift, lift]
        self.drum_lift_pub.publish(float_array)

    def set_drum_spin_rate(self, spin: float):
        """ Sets the drum spin rate
        :param spin: The desired spin rate, between -1 and 1 inclusive.
        """
        float_array = Float64MultiArray()
        float_array.data = [spin]
        self.drum_spin_pub.publish(float_array)

    def skip_if_deprioritised(self, control_source: ControlSource) -> bool:
        self.set_last_message_time()
        current_time = time.time()
        # If this message source is prioritised, don't skip
        if control_source == self.prioritised_source:
            self.last_priority_message_time = current_time
            return False

        prioritised_message_received_recently = current_time - self.last_priority_message_time < self.priority_timeout_seconds
        if prioritised_message_received_recently:
            self.get_logger().info(f"Skipping {control_source} message as {self.prioritised_source} message received recently")
        return prioritised_message_received_recently

    def send_zero_if_no_recent_messages(self):
        if time.time() - self.last_message_received_time >= self.zero_timer_threshold_seconds:
            # Spin is a velocity command, so "no command" means stop spinning.
            self.set_drum_spin_rate(0.0)
            # Lift is a closed-loop *position* on the hardware: re-publishing here
            # would actively drive the drum (0.5 sends it to mid-travel, which would
            # drop a raised bucket during deposition, where lift is never commanded).
            # "No command" means hold position, so we leave the last lift target
            # latched in the position controller / hardware servo and send nothing.

# Unused, as clamped at control level
def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

def normalise_for_actuator_convention(value):
    """ Converts from -1 to 1 range to 0 to 1 range, as required by linear actuators. """
    return value / 2 + 0.5

def main():
    rclpy.init()
    node = DrumInterface()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
