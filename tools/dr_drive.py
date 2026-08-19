#!/usr/bin/env python3
"""
Open-loop scripted drive for dead-reckoning testing (sim only).

Drives a closed rectangle inside arena_april.world, sized and placed to keep the
rover clear of every wall, and publishes on /cmd_vel_nav (the drive mux's nav
input) so no Nav2 or teleop is needed.

EVERYTHING IS TIMED ON THE SIM CLOCK, not wall time. Visual odometry costs real
CPU and drags the real-time factor down, so a wall-clock pattern would drive a
SHORTER path in the VO-on runs and the two conditions would not be comparable.
On the sim clock both conditions receive an identical command sequence.

Geometry (rover spawns at (-1.2, -2.95) facing +x; arena is x +/-2.2, y +/-3.95):
    east 1.6 m -> north 2.0 m -> west 1.6 m -> south 2.0 m, counter-clockwise.
    Path stays inside x [-1.2, 0.4], y [-2.95, -0.95]; nearest wall is 0.95 m
    from the path, and the rover's half-diagonal is 0.64 m.

    python3 tools/dr_drive.py --laps 1
"""
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

V = 0.20    # m/s
W = 0.40    # rad/s

import math

def leg(dist):
    return (V, 0.0, dist / V)

def turn(deg):
    return (0.0, math.copysign(W, deg), math.radians(abs(deg)) / W)

# (linear m/s, angular rad/s, sim-seconds)
SETTLE = [(0.0, 0.0, 3.0)]
LAP = [
    leg(1.6), turn(90),
    leg(2.0), turn(90),
    leg(1.6), turn(90),
    leg(2.0), turn(90),
]


class DrivePattern(Node):
    def __init__(self):
        super().__init__("dr_drive")
        self.pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def wait_for_clock(self):
        # /clock has to be ticking before any sim-time deadline means anything.
        while rclpy.ok() and self.now() == 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self, segments):
        for lin, ang, secs in segments:
            end = self.now() + secs
            while rclpy.ok() and self.now() < end:
                msg = Twist()
                msg.linear.x = float(lin)
                msg.angular.z = float(ang)
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.02)
        for _ in range(10):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--laps", type=int, default=1)
    args = ap.parse_args()

    rclpy.init()
    node = DrivePattern()
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    node.wait_for_clock()

    segments = SETTLE + LAP * args.laps + SETTLE
    total = sum(s[2] for s in segments)
    node.get_logger().info(f"driving {args.laps} lap(s), {total:.1f} sim-seconds")
    try:
        node.run(segments)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    node.get_logger().info("drive complete")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
