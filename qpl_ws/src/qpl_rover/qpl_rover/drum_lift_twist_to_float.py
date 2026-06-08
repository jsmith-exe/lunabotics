#!/usr/bin/env python3
"""
Forwards a Float64 position setpoint [0.0, 1.0] to the drum-lift position
controller.  0.0 = fully down, 1.0 = fully up.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class DrumLiftPositionSetter(Node):
    def __init__(self):
        super().__init__("drum_lift_float_bridge")

        self.cmd_sub = self.create_subscription(
            Float64, "/drum_lift_cont/cmd_pos", self.cmd_cb, 10
        )
        self.pub = self.create_publisher(
            Float64MultiArray, "/drum_lift_cont/commands", 10
        )

    def cmd_cb(self, msg: Float64):
        position = max(0.0, min(1.0, msg.data))
        out = Float64MultiArray()
        out.data = [position]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DrumLiftPositionSetter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
