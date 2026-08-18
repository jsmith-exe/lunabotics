#!/usr/bin/env python3
"""
Scripted open-loop drive for localisation testing (sim only).

Publishes a repeating pattern on /cmd_vel_nav (the drive mux's nav input) so the
rover moves through tag-visible and tag-blind orientations without needing Nav2:
forward, turn left, forward, turn right, pause — then repeats.

    python3 tools/drive_pattern.py --duration 60
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# (linear m/s, angular rad/s, seconds)
PATTERN = [
    (0.15, 0.0, 6.0),
    (0.0, 0.3, 5.0),
    (0.15, 0.0, 6.0),
    (0.0, -0.3, 5.0),
    (-0.10, 0.0, 4.0),
    (0.0, 0.0, 2.0),
]


class DrivePattern(Node):
    def __init__(self):
        super().__init__("drive_pattern")
        self.pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)

    def run(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            for lin, ang, secs in PATTERN:
                seg_end = time.monotonic() + secs
                while time.monotonic() < min(seg_end, end):
                    msg = Twist()
                    msg.linear.x = lin
                    msg.angular.z = ang
                    self.pub.publish(msg)
                    time.sleep(0.1)
                if time.monotonic() >= end:
                    break
        self.pub.publish(Twist())  # stop


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = DrivePattern()
    try:
        node.run(args.duration)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
