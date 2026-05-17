#!/usr/bin/env python3
"""
Bridges Twist -> Float64MultiArray for the drum-lift velocity controller.
Reads linear.z from the incoming Twist and forwards it as the velocity
command for drum_joint, clamping to zero when the joint is at a limit.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

DRUM_JOINT_NAME = "drum_joint"
UPPER_LIMIT = 0.0
LOWER_LIMIT = -0.1
DEADBAND = 0.005


class DrumLiftTwistToFloat(Node):
    def __init__(self):
        super().__init__("drum_lift_twist_to_float")

        # Default to upper limit — drum starts retracted, so upward is blocked
        # until joint_states arrives and proves otherwise.
        self.drum_pos = UPPER_LIMIT

        self.cmd_sub = self.create_subscription(
            Twist, "/drum_lift_cont/cmd_vel_unstamped", self.cmd_cb, 10
        )
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self.js_cb, 10
        )
        self.pub = self.create_publisher(
            Float64MultiArray, "/drum_lift_cont/commands", 10
        )

    def js_cb(self, msg: JointState):
        try:
            idx = msg.name.index(DRUM_JOINT_NAME)
        except ValueError:
            return
        self.drum_pos = msg.position[idx]

    def cmd_cb(self, msg: Twist):
        velocity = msg.linear.z

        if self.drum_pos >= UPPER_LIMIT - DEADBAND and velocity > 0.0:
            velocity = 0.0
        elif self.drum_pos <= LOWER_LIMIT + DEADBAND and velocity < 0.0:
            velocity = 0.0

        out = Float64MultiArray()
        out.data = [velocity]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DrumLiftTwistToFloat()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
