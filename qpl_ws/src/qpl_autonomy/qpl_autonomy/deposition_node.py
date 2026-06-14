#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from qpl_autonomy.navigation_manager import NavigationManager
from qpl_autonomy.waypoint_manager import WaypointManager

# ── Tunable parameters ────────────────────────────────────────────────────────
DRUM_SPIN_SPEED = -1.0   # spin command, -1..1 (reverse for deposition)

# Start/finish poses come from config/waypoints.yaml (single source of truth).
# construction_zone   → start of the dump pass
# construction_finish → end of the dump pass
# ─────────────────────────────────────────────────────────────────────────────


class DepositionNode(Node):

    def __init__(self):
        super().__init__("deposition_node")

        self.declare_parameter("config_path", "")
        config_path = self.get_parameter(
            "config_path"
        ).get_parameter_value().string_value

        self._waypoints = WaypointManager(config_path)
        self._start  = self._waypoints.get_waypoint("construction_zone")
        self._finish = self._waypoints.get_waypoint("construction_finish")

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
            self._nav.navigate_to(self._start)
            self._enter("WAIT_START")

        elif self._state == "WAIT_START":
            self._spin(0.0)
            if self._nav.goal_complete:
                self.get_logger().info("Reached construction zone — spinning reverse")
                self._spin(DRUM_SPIN_SPEED)
                self._nav.navigate_to(self._finish)
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
