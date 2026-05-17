#!/usr/bin/env python3
"""
Bridges Twist -> Float64MultiArray for the drum-spin velocity controller.

Reads angular.y from the incoming Twist and publishes it as the velocity
command for drum_spin_joint. No joint-limit clamping needed — the spin
joint is continuous.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class DrumTwistToFloat(Node):
    def __init__(self):
        super().__init__("drum_twist_to_float")
        self.sub = self.create_subscription(
            Twist, "/drum_cont/cmd_vel_unstamped", self.cb, 10
        )
        self.pub = self.create_publisher(
            Float64MultiArray, "/drum_cont/commands", 10
        )

    def cb(self, msg: Twist):
        out = Float64MultiArray()
        out.data = [msg.angular.y]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DrumTwistToFloat()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()