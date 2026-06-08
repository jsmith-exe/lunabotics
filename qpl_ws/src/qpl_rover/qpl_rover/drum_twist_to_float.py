#!/usr/bin/env python3
"""
Forwards a Float64 velocity command to the drum-spin velocity controller.
Subscribes to /drum_cmd (Float64) and publishes to /drum_cont/commands
(Float64MultiArray) as required by ros2_control.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class DrumFloatBridge(Node):
    def __init__(self):
        super().__init__("drum_float_bridge")
        self.sub = self.create_subscription(
            Float64, "/drum_cmd", self.cb, 10
        )
        self.pub = self.create_publisher(
            Float64MultiArray, "/drum_cont/commands", 10
        )

    def cb(self, msg: Float64):
        out = Float64MultiArray()
        out.data = [msg.data]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DrumFloatBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
