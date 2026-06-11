#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from qpl_autonomy.navigation_manager import NavigationManager

# ── Tunable parameters ────────────────────────────────────────────────────────
DRUM_SPIN_SPEED  =  5.0   # rad/s, forward spin

LOWER_DURATION_S =  3.0   # seconds to wait for drum to reach down position
RAISE_DURATION_S =  3.0   # seconds to wait for drum to reach up position

EXCAVATION_START  = {"x": 1.7, "y": 3.0, "yaw": 0.0}
EXCAVATION_FINISH = {"x": 3.7, "y": 3.0, "yaw": 0.0}  # 2 m further on x
# ─────────────────────────────────────────────────────────────────────────────


class ExcavationNode(Node):

    def __init__(self):
        super().__init__("excavation_node")

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
            self._nav.navigate_to(EXCAVATION_START)
            self._enter("WAIT_START")

        elif self._state == "WAIT_START":
            self._lift(1.0)  # hold up while navigating
            if self._nav.goal_complete:
                self.get_logger().info("Reached excavation start — spinning up")
                self._spin(DRUM_SPIN_SPEED)
                self._enter("LOWERING")

        elif self._state == "LOWERING":
            self._lift(0.0)  # command fully down
            if self._elapsed() >= LOWER_DURATION_S:
                self._nav.navigate_to(EXCAVATION_FINISH)
                self._enter("EXCAVATING")

        elif self._state == "EXCAVATING":
            self._lift(0.0)  # hold fully down while driving
            if self._nav.goal_complete:
                self.get_logger().info("Reached excavation finish — stopping drum")
                self._spin(0.0)
                self._enter("RAISING")

        elif self._state == "RAISING":
            self._lift(1.0)  # command fully up
            if self._elapsed() >= RAISE_DURATION_S:
                self._enter("DONE")

        elif self._state == "DONE":
            self._lift(1.0)  # hold fully up
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
