#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from qpl_autonomy.navigation_manager import NavigationManager

# ── Tunable parameters ────────────────────────────────────────────────────────
DRUM_SPIN_SPEED = -5.0   # rad/s, reverse spin for deposition

DEPOSITION_START  = {"x": 1.2, "y": -2.5, "yaw": 0.0}
DEPOSITION_FINISH = {"x": 1.2, "y": -3.5, "yaw": 0.0}  # 1 m lower on y
# ─────────────────────────────────────────────────────────────────────────────


class DepositionNode(Node):

    def __init__(self):
        super().__init__("deposition_node")

        self._drum_pub = self.create_publisher(
            Float64, "/drum_spin_control/autonomy", 10
        )

        self._nav = NavigationManager(self)

        self._state = "NAVIGATE_TO_START"
        self._timer = self.create_timer(0.1, self._loop)
        self.get_logger().info("Deposition node started")

    # ── helpers ───────────────────────────────────────────────────────────────

    def _enter(self, state: str):
        self._state = state
        self.get_logger().info(f"State: {state}")

    def _spin(self, speed: float):
        self._drum_pub.publish(Float64(data=speed))

    # ── FSM ───────────────────────────────────────────────────────────────────

    def _loop(self):

        if self._state == "NAVIGATE_TO_START":
            self._spin(0.0)
            self._nav.navigate_to(DEPOSITION_START)
            self._enter("WAIT_START")

        elif self._state == "WAIT_START":
            self._spin(0.0)
            if self._nav.goal_complete:
                self.get_logger().info("Reached construction zone — spinning reverse")
                self._spin(DRUM_SPIN_SPEED)
                self._nav.navigate_to(DEPOSITION_FINISH)
                self._enter("DEPOSITING")

        elif self._state == "DEPOSITING":
            self._spin(DRUM_SPIN_SPEED)
            if self._nav.goal_complete:
                self.get_logger().info("Reached deposition finish — stopping drum")
                self._spin(0.0)
                self._enter("DONE")

        elif self._state == "DONE":
            self._spin(0.0)
            self.get_logger().info("Deposition complete")
            self._timer.cancel()


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DepositionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
