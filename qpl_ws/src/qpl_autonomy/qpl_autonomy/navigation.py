import time
from math import cos, sin

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration

# How long to wait for a cancelled goal to report back before sending the next goal anyway
CANCEL_SETTLE_S = 3.0


class Navigation:
    """Sends Nav2 navigate_to_pose goals without blocking the autonomy node.

    Never waits for Nav2 at startup, so the blind sequences work without it running.
    Poll `done` / `succeeded` from the node's timer after calling navigate_to(), and only
    call navigate_to() when ready() is true.
    """

    def __init__(self, node, goal_timeout_s):
        self.node = node
        self.goal_timeout = Duration(seconds=goal_timeout_s)
        self.client = ActionClient(node, NavigateToPose, "navigate_to_pose")

        self.done = False
        self.succeeded = False
        self._active = False
        self._deadline = None
        self._goal_handle = None

        # Each goal gets a new generation. Callbacks for an older goal are ignored, so
        # a cancelled goal's late CANCELED result can't be taken as the new goal's.
        self._generation = 0

        # Generations of cancelled goals that haven't reported a result yet -> give-up time.
        # A goal sent while Nav2 is still cancelling the last one gets aborted, which used to
        # count as a second failure.
        self._unsettled = {}

    def is_available(self):
        return self.client.server_is_ready()

    def ready(self):
        """True once every cancelled goal has finished, so a new goal won't race the cancel."""
        now = time.monotonic()
        for generation, give_up in list(self._unsettled.items()):
            if now >= give_up:
                self.node.get_logger().warning("A cancelled navigation goal never reported back. Carrying on.")
                del self._unsettled[generation]
        return not self._unsettled

    def navigate_to(self, x, y, yaw):
        self._generation += 1
        generation = self._generation
        self.done = False
        self.succeeded = False
        self._active = True
        self._goal_handle = None
        self._deadline = self.node.get_clock().now() + self.goal_timeout

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = sin(yaw / 2.0)
        pose.pose.orientation.w = cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._goal_response(f, generation))

    def update(self):
        # Call every tick: gives up on a goal that runs past the timeout
        if self._active and self.node.get_clock().now() >= self._deadline:
            self.node.get_logger().warning("Navigation goal timed out. Cancelling.")
            self._finish(False)
            self._cancel(self._generation)

    def cancel(self):
        # Cancel any active goal; safe to call when nothing is running
        was_active = self._active
        generation = self._generation
        self._generation += 1
        self._active = False
        if was_active:
            self._cancel(generation)

    def _goal_response(self, future, generation):
        handle = future.result()
        if generation != self._generation or not self._active:
            # Cancelled or timed out before Nav2 answered: stop it, and stay not-ready until it has
            if handle.accepted:
                handle.cancel_goal_async()
                handle.get_result_async().add_done_callback(lambda f: self._result(f, generation))
            else:
                self._unsettled.pop(generation, None)
            return

        if not handle.accepted:
            self.node.get_logger().error("Navigation goal rejected by Nav2.")
            self._finish(False)
            return

        self._goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._result(f, generation))

    def _result(self, future, generation):
        self._unsettled.pop(generation, None)
        if generation != self._generation or not self._active:
            return
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().warning(f"Navigation goal did not succeed (status {status}).")
        self._finish(status == GoalStatus.STATUS_SUCCEEDED)

    def _finish(self, succeeded):
        self._active = False
        self.succeeded = succeeded
        self.done = True

    def _cancel(self, generation):
        # Not ready() until this goal reports back. With no handle yet, _goal_response
        # cancels it when Nav2 answers.
        self._unsettled[generation] = time.monotonic() + CANCEL_SETTLE_S
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
