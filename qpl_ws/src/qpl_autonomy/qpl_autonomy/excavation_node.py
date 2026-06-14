#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from qpl_autonomy.navigation_manager import NavigationManager
from qpl_autonomy.waypoint_manager import WaypointManager

# ── Tunable parameters ────────────────────────────────────────────────────────
DRUM_SPIN_SPEED  =  1.0   # spin command, -1..1 (forward)

# Lift commands are in the -1..1 convention consumed by drum_command_interface
# (normalised v/2+0.5 → position 0..1). -1 and +1 are the two extremes; 0 is mid.
LIFT_DOWN        = -1.0   # drum fully lowered
LIFT_UP          =  1.0   # drum fully raised

LOWER_DURATION_S =  3.0   # seconds to wait for drum to reach down position
RAISE_DURATION_S =  3.0   # seconds to wait for drum to reach up position

# Start/finish poses come from config/waypoints.yaml (single source of truth).
# excavation_zone   → start of the dig pass
# excavation_finish → end of the dig pass
# ─────────────────────────────────────────────────────────────────────────────


class ExcavationNode(Node):

    def __init__(self):
        super().__init__("excavation_node")

        self.declare_parameter("config_path", "")
        config_path = self.get_parameter(
            "config_path"
        ).get_parameter_value().string_value

        self._waypoints = WaypointManager(config_path)
        self._start  = self._waypoints.get_waypoint("excavation_zone")
        self._finish = self._waypoints.get_waypoint("excavation_finish")

        self._drum_pub = self.create_publisher(
            Float64, "/drum_spin_control/autonomy", 10
        )
        self._lift_pub = self.create_publisher(
            Float64, "/drum_lift_control/autonomy", 10
        )

        self._nav = NavigationManager(self)

        self._state = "NAVIGATE_TO_START"
        self._state_entry = None

        self._timer = self.create_timer(0.1, self._loop)
        self.get_logger().info("Excavation node started")

    # ── helpers ───────────────────────────────────────────────────────────────

    def _enter(self, state: str):
        self._state = state
        self._state_entry = self.get_clock().now()
        self.get_logger().info(f"State: {state}")

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._state_entry).nanoseconds / 1e9

    def _spin(self, speed: float):
        self._drum_pub.publish(Float64(data=speed))

    def _lift(self, position: float):
        self._lift_pub.publish(Float64(data=position))

    # ── FSM ───────────────────────────────────────────────────────────────────

    def _loop(self):

        if self._state == "NAVIGATE_TO_START":
            self._nav.navigate_to(self._start)
            self._enter("WAIT_START")

        elif self._state == "WAIT_START":
            self._lift(LIFT_UP)  # hold up while navigating
            if self._nav.goal_complete:
                self.get_logger().info("Reached excavation start — spinning up")
                self._spin(DRUM_SPIN_SPEED)
                self._enter("LOWERING")

        elif self._state == "LOWERING":
            self._lift(LIFT_DOWN)  # command fully down
            if self._elapsed() >= LOWER_DURATION_S:
                self._nav.navigate_to(self._finish)
                self._enter("EXCAVATING")

        elif self._state == "EXCAVATING":
            self._lift(LIFT_DOWN)  # hold fully down while driving
            if self._nav.goal_complete:
                self.get_logger().info("Reached excavation finish — stopping drum")
                self._spin(0.0)
                self._enter("RAISING")

        elif self._state == "RAISING":
            self._lift(LIFT_UP)  # command fully up
            if self._elapsed() >= RAISE_DURATION_S:
                self._enter("DONE")

        elif self._state == "DONE":
            self._lift(LIFT_UP)  # hold fully up
            self.get_logger().info("Excavation complete")
            self._timer.cancel()


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ExcavationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
