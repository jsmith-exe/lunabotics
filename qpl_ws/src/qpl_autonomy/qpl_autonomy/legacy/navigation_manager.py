from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


class NavigationManager:

    def __init__(self, node, goal_timeout=120.0):

        self.node = node
        self.goal_timeout = goal_timeout

        self.nav_client = ActionClient(
            node,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Terminal flags read by the FSMs. goal_complete becomes True once the
        # goal reaches ANY terminal outcome (success, abort, reject, timeout);
        # goal_succeeded tells them which it was so they don't act on a failure.
        self.goal_complete = False
        self.goal_succeeded = False
        self.goal_handle = None

        # Internal: an in-flight goal is "active" until it terminates. Used to
        # ignore late callbacks after a timeout/cancel and to drive the watchdog.
        self._active = False
        self._deadline = None

        # Don't block forever if Nav2 isn't up yet — log so it's diagnosable.
        while not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().warn(
                "Waiting for the navigate_to_pose action server (is Nav2 running?)"
            )

        # Watchdog: catches goals that are accepted but never return a result.
        self._watchdog_timer = self.node.create_timer(0.5, self._watchdog)

    # =====================================================
    # SEND GOAL
    # =====================================================

    def navigate_to(self, waypoint):

        self.goal_complete = False
        self.goal_succeeded = False
        self._active = True
        self._deadline = self.node.get_clock().now() + Duration(seconds=self.goal_timeout)

        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()

        pose.pose.position.x = float(waypoint["x"])
        pose.pose.position.y = float(waypoint["y"])

        q = quaternion_from_euler(0, 0, float(waypoint["yaw"]))

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal_msg.pose = pose

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    # =====================================================
    # RESPONSE
    # =====================================================

    def goal_response_callback(self, future):

        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error("Goal rejected")
            # Surface a failed terminal result so the FSM retries/holds instead
            # of hanging in its WAIT state forever.
            self._active = False
            self.goal_succeeded = False
            self.goal_complete = True
            return

        self.node.get_logger().info("Goal accepted")

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # =====================================================
    # RESULT
    # =====================================================

    def result_callback(self, future):

        # Ignore a late result for a goal we already gave up on (timeout/cancel).
        if not self._active:
            return

        status = future.result().status
        self.goal_succeeded = (status == GoalStatus.STATUS_SUCCEEDED)

        if self.goal_succeeded:
            self.node.get_logger().info("Goal succeeded")
        else:
            self.node.get_logger().warn(f"Goal did not succeed (status {status})")

        self._active = False
        self.goal_complete = True

    # =====================================================
    # WATCHDOG
    # =====================================================

    def _watchdog(self):

        if not self._active or self._deadline is None:
            return

        if self.node.get_clock().now() >= self._deadline:
            self.node.get_logger().warn(
                f"Navigation goal exceeded {self.goal_timeout:.0f}s timeout — cancelling"
            )
            self.cancel_goal()
            self._active = False
            self.goal_succeeded = False
            self.goal_complete = True

    # =====================================================
    # CANCEL
    # =====================================================

    def cancel_goal(self):

        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
