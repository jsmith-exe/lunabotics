#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped


ACTIVE_STATUSES = (
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
)


class AprilTagGate(Node):
    """Rate-limits AprilTag fixes into the global EKF while Nav2 is driving.

    Every fix moves map -> odom, so at camera frame rate the map frame keeps
    shifting under the planner and controller mid-goal. While a navigate_to_pose
    goal is active, at most one fix per `min_period_active` seconds is passed on.
    With no goal active every fix passes, so the rover localises quickly when idle.
    """

    def __init__(self):
        super().__init__('apriltag_gate')

        self._min_period = self.declare_parameter('min_period_active', 5.0).value
        self._nav_active = False
        self._last_forward_time = None

        self._pub = self.create_publisher(PoseWithCovarianceStamped, '/apriltag/pose_gated', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/apriltag/pose', self._pose_cb, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._status_cb, 10)

    def _status_cb(self, msg: GoalStatusArray):
        # The array also lists finished goals for a while, so check every entry.
        active = any(s.status in ACTIVE_STATUSES for s in msg.status_list)
        if active != self._nav_active:
            self._nav_active = active
            if active:
                self.get_logger().info(
                    f'Nav2 goal active: AprilTag fixes limited to one per {self._min_period:.1f} s')
            else:
                self.get_logger().info('Nav2 idle: passing every AprilTag fix')

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        now = self.get_clock().now()
        if self._nav_active and self._last_forward_time is not None:
            if (now - self._last_forward_time).nanoseconds / 1e9 < self._min_period:
                return
        self._last_forward_time = now
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = AprilTagGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
