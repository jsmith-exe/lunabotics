#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist


class TeleopWatchdog(Node):
    """Warns when /cmd_vel_teleop goes quiet for longer than `timeout` seconds.

    The drive mux zeroes teleop output after 0.25 s of silence, so a gap
    above 0.2 s means the rover is about to stop (or stutter) on its own.

    This is for testing an issue where the network would seem to drop temporarily.
    """

    def __init__(self):
        super().__init__('teleop_watchdog')

        self._timeout = Duration(
            seconds=self.declare_parameter('timeout', 0.2).value)
        self._last_msg_time = None
        self._stale = False

        self.create_subscription(Twist, '/cmd_vel_teleop', self._cmd_cb, 10)
        # Check at 4x the timeout rate so a gap is reported promptly.
        self.create_timer(self._timeout.nanoseconds / 1e9 / 4, self._check)

    def _cmd_cb(self, _msg: Twist):
        now = self.get_clock().now()
        if self._stale:
            gap = (now - self._last_msg_time).nanoseconds / 1e9
            self.get_logger().info(f'/cmd_vel_teleop resumed after {gap:.2f} s')
            self._stale = False
        self._last_msg_time = now

    def _check(self):
        if self._last_msg_time is None:
            self.get_logger().warn(
                'No commands received on /cmd_vel_teleop yet',
                throttle_duration_sec=5.0)
            return

        gap = self.get_clock().now() - self._last_msg_time
        if gap > self._timeout:
            self._stale = True
            self.get_logger().warn(
                f'No commands on /cmd_vel_teleop for {gap.nanoseconds / 1e9:.2f} s',
                throttle_duration_sec=1.0)


def main():
    rclpy.init()
    rclpy.spin(TeleopWatchdog())


if __name__ == '__main__':
    main()
