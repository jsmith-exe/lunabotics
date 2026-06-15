#!/usr/bin/env python3
"""
Full-autonomy mission orchestrator.

Runs a complete excavation pass, then a deposition pass, and loops the cycle
until the node is stopped (Ctrl-C / killed). It reuses the tuned per-phase
parameters from excavation_node and deposition_node so there is a single source
of truth, and drives the same drum/lift command topics and Nav2 waypoints those
standalone nodes use.

Sequence per cycle:
  excavation: drive to dig start → lower drum → dig pass (drum spinning) → raise
  deposition: drive to dump zone (drum held raised) → reverse-spin dump pass
then repeat.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from qpl_autonomy.navigation_manager import NavigationManager
from qpl_autonomy.waypoint_manager import WaypointManager

# Single source of truth for the tuned per-phase parameters.
from qpl_autonomy.excavation_node import (
    DRUM_SPIN_SPEED as DIG_SPIN,     # forward spin while digging
    LIFT_DOWN,
    LIFT_UP,
    LIFT_DOWN_POS,
    LIFT_UP_POS,
    LIFT_REACHED_TOL,
    LOWER_DURATION_S,
    RAISE_DURATION_S,
    LIFT_JOINTS,
)
from qpl_autonomy.deposition_node import DRUM_SPIN_SPEED as DUMP_SPIN  # reverse spin while dumping


class FullAutonomyNode(Node):

    def __init__(self):
        super().__init__("full_autonomy_node")

        self.declare_parameter("config_path", "")
        config_path = self.get_parameter(
            "config_path"
        ).get_parameter_value().string_value

        self._waypoints = WaypointManager(config_path)
        self._exc_start  = self._waypoints.get_waypoint("excavation_zone")
        self._exc_finish = self._waypoints.get_waypoint("excavation_finish")
        self._dep_start  = self._waypoints.get_waypoint("construction_zone")
        self._dep_finish = self._waypoints.get_waypoint("construction_finish")

        self._drum_pub = self.create_publisher(
            Float64, "/drum_spin_control/autonomy", 10
        )
        self._lift_pub = self.create_publisher(
            Float64, "/drum_lift_control/autonomy", 10
        )

        # Measured lift position per actuator joint, populated from /joint_states.
        self._lift_positions = {}
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        self._nav = NavigationManager(self)

        self._cycle = 1
        self._state = "EXC_NAV_START"
        self._state_entry = self.get_clock().now()

        self._timer = self.create_timer(0.1, self._loop)
        self.get_logger().info(
            "Full autonomy node started — looping excavation → deposition until stopped"
        )

    # ── helpers ───────────────────────────────────────────────────────────────

    def _enter(self, state: str):
        self._state = state
        self._state_entry = self.get_clock().now()
        self.get_logger().info(f"[cycle {self._cycle}] State: {state}")

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._state_entry).nanoseconds / 1e9

    def _spin(self, speed: float):
        self._drum_pub.publish(Float64(data=speed))

    def _lift(self, position: float):
        self._lift_pub.publish(Float64(data=position))

    def _joint_state_cb(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in LIFT_JOINTS:
                self._lift_positions[name] = position

    def _lift_reached(self, target_pos: float) -> bool:
        """True once every tracked lift joint reports within tolerance of target_pos.
        Returns False until feedback for all joints has been seen, so the caller
        falls back to its timeout."""
        if not all(j in self._lift_positions for j in LIFT_JOINTS):
            return False
        return all(
            abs(self._lift_positions[j] - target_pos) <= LIFT_REACHED_TOL
            for j in LIFT_JOINTS
        )

    # ── FSM ───────────────────────────────────────────────────────────────────

    def _loop(self):

        # ── Excavation phase ────────────────────────────────────────────────
        if self._state == "EXC_NAV_START":
            self._nav.navigate_to(self._exc_start)
            self._enter("EXC_WAIT_START")

        elif self._state == "EXC_WAIT_START":
            self._lift(LIFT_UP)  # hold up while navigating
            if self._nav.goal_complete:
                self.get_logger().info("Reached excavation start — spinning up")
                self._spin(DIG_SPIN)
                self._enter("EXC_LOWERING")

        elif self._state == "EXC_LOWERING":
            self._lift(LIFT_DOWN)  # command fully down (servo stops at the end-stop)
            reached = self._lift_reached(LIFT_DOWN_POS)
            if reached or self._elapsed() >= LOWER_DURATION_S:
                why = "drum down" if reached else "lower timeout"
                self.get_logger().info(f"Starting dig pass ({why})")
                self._nav.navigate_to(self._exc_finish)
                self._enter("EXC_EXCAVATING")

        elif self._state == "EXC_EXCAVATING":
            self._lift(LIFT_DOWN)  # hold fully down while driving
            if self._nav.goal_complete:
                self.get_logger().info("Reached excavation finish — stopping drum")
                self._spin(0.0)
                self._enter("EXC_RAISING")

        elif self._state == "EXC_RAISING":
            self._lift(LIFT_UP)  # command fully up (servo stops at the end-stop)
            if self._lift_reached(LIFT_UP_POS) or self._elapsed() >= RAISE_DURATION_S:
                self._enter("DEP_NAV_START")

        # ── Deposition phase (drum held raised throughout) ──────────────────
        elif self._state == "DEP_NAV_START":
            self._lift(LIFT_UP)
            self._spin(0.0)
            self._nav.navigate_to(self._dep_start)
            self._enter("DEP_WAIT_START")

        elif self._state == "DEP_WAIT_START":
            self._lift(LIFT_UP)
            self._spin(0.0)
            if self._nav.goal_complete:
                self.get_logger().info("Reached construction zone — spinning reverse")
                self._spin(DUMP_SPIN)
                self._nav.navigate_to(self._dep_finish)
                self._enter("DEP_DEPOSITING")

        elif self._state == "DEP_DEPOSITING":
            self._lift(LIFT_UP)
            self._spin(DUMP_SPIN)
            if self._nav.goal_complete:
                self.get_logger().info("Reached deposition finish — stopping drum")
                self._spin(0.0)
                self.get_logger().info(f"=== Cycle {self._cycle} complete — looping ===")
                self._cycle += 1
                self._enter("EXC_NAV_START")


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FullAutonomyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
