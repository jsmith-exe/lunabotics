"""
Prioritises subscriptions from teleop and autonomy control, automatically stops the drum and holds the lift on
 absence of data, and formats data for suitable publishing.

A lift command of 0.0 (or NaN) means hold, as it always has on the rover, where the CAN driver stops the actuators
 on 0.0. In the sim 0.0 is a position the bucket drives down to, so a hold is sent as the lift's position when the
 hold started instead, from /joint_states. Other lift commands are clamped to the actuators' range in
 diffdrive_canbus' can_config.yaml, which the sim would otherwise reach and the rover never could.
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from enum import Enum, auto

LIFT_JOINTS = ("left_linear_actuator_joint", "right_linear_actuator_joint")
# Lift feedback older than this isn't held at; 0.0, the CAN driver's stop, is sent instead
LIFT_FEEDBACK_MAX_AGE_S = 1.0


def load_lift_limits_m():
    """ (min, max) drum lift in metres, from diffdrive_canbus' can_config.yaml. """
    path = os.path.join(get_package_share_directory("diffdrive_canbus"), "can_config.yaml")
    with open(path) as file:
        drum = yaml.safe_load(file)["drum"]
    return drum["min_lift_mm"] / 1000, drum["max_lift_mm"] / 1000

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

        self.lift_min_m, self.lift_max_m = load_lift_limits_m()
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self.lift_position_m = None # Mean of the two actuators
        self.lift_position_time = 0.0
        self.hold_position_m = None # Set while holding

        # Automatically stop the drum and hold the lift if nothing received.
        # Timed on the node clock, not wall time: in the sim the autonomy node publishes on sim
        # time, which runs slower than wall time, so a wall-clock timeout fired between its
        # messages and kept zeroing the lift and spin.
        self.zero_timer_check_rate = 5 # Hz
        # 0.5, not 0.3: teleop and autonomy both publish at 5 Hz, so 0.3 s was 1.5 periods and
        # tripped on any jitter.
        self.zero_timer_threshold_seconds = 0.5 # How long there must be no commands before stopping
        self.zero_timer = self.create_timer(1/self.zero_timer_check_rate, self.send_zero_if_no_recent_messages)
        self.last_message_received_time = 0

        self.prioritised_source = ControlSource.TELEOP
        self.last_priority_message_time = 0
        self.priority_timeout_seconds = 3 # How long to wait before accepting deprioritised messages
        self.get_logger().info(
            f"Drum command interface node initialised, lift limits {self.lift_min_m:.3f}-{self.lift_max_m:.3f} m")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def set_last_message_time(self):
        self.last_message_received_time = self.now_s()

    def lift_autonomy_cb(self, msg: Float64):
        if self.skip_if_deprioritised(ControlSource.AUTONOMY): return
        self.set_drum_lift_pos_m(msg.data)

    def spin_autonomy_cb(self, msg: Float64):
        if self.skip_if_deprioritised(ControlSource.AUTONOMY): return
        self.set_drum_spin_rate(msg.data)

    def lift_teleop_cb(self, msg: Float64):
        if self.skip_if_deprioritised(ControlSource.TELEOP): return
        self.set_drum_lift_pos_m(msg.data)

    def spin_teleop_cb(self, msg: Float64):
        if self.skip_if_deprioritised(ControlSource.TELEOP): return
        self.set_drum_spin_rate(msg.data)

    def joint_state_cb(self, msg: JointState):
        positions = [position for name, position in zip(msg.name, msg.position) if name in LIFT_JOINTS]
        if len(positions) == len(LIFT_JOINTS):
            self.lift_position_m = sum(positions) / len(positions)
            self.lift_position_time = self.now_s()

    def set_drum_lift_pos_m(self, lift: float):
        """ Sets the drum lift position in metres.
        :param lift: The desired height, in metres - clamped to the limits in can_config.yaml. 0.0 or NaN holds.
        """
        if lift == 0.0 or math.isnan(lift):
            lift = self.hold_position_m_or_stop()
        else:
            self.hold_position_m = None
            lift = clamp(lift, self.lift_min_m, self.lift_max_m)
        float_array = Float64MultiArray()
        float_array.data = [lift, lift]
        self.drum_lift_pub.publish(float_array)

    def hold_position_m_or_stop(self) -> float:
        """ The lift position when the hold started, clamped; 0.0 (the CAN driver's stop) without recent feedback. """
        if self.hold_position_m is None:
            if self.lift_position_m is None or self.now_s() - self.lift_position_time > LIFT_FEEDBACK_MAX_AGE_S:
                return 0.0
            self.hold_position_m = clamp(self.lift_position_m, self.lift_min_m, self.lift_max_m)
        return self.hold_position_m

    def set_drum_spin_rate(self, spin: float):
        """ Sets the drum spin rate
        :param spin: The desired spin rate, between -1 and 1 inclusive.
        """
        float_array = Float64MultiArray()
        float_array.data = [spin]
        self.drum_spin_pub.publish(float_array)

    def skip_if_deprioritised(self, control_source: ControlSource) -> bool:
        self.set_last_message_time()
        current_time = self.now_s()
        # If this message source is prioritised, don't skip
        if control_source == self.prioritised_source:
            self.last_priority_message_time = current_time
            return False

        prioritised_message_received_recently = current_time - self.last_priority_message_time < self.priority_timeout_seconds
        if prioritised_message_received_recently:
            self.get_logger().info(f"Skipping {control_source} message as {self.prioritised_source} message received recently")
        return prioritised_message_received_recently

    def send_zero_if_no_recent_messages(self):
        if self.now_s() - self.last_message_received_time >= self.zero_timer_threshold_seconds:
            self.set_drum_spin_rate(0.0)
            self.set_drum_lift_pos_m(0.0) # hold

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

def main():
    rclpy.init()
    node = DrumInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
