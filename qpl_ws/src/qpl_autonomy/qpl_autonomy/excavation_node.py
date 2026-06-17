#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from qpl_autonomy.navigation_manager import NavigationManager
from qpl_autonomy.waypoint_manager import WaypointManager

# ── Tunable parameters ────────────────────────────────────────────────────────
DRUM_SPIN_SPEED  =  1.0   # spin command, -1..1 (forward)

# Lift commands are in the -1..1 convention consumed by drum_command_interface
# (normalised v/2+0.5 → position 0..1). -1 and +1 are the two extremes; 0 is mid.
LIFT_DOWN        = -1.0   # drum fully lowered
LIFT_UP          =  1.0   # drum fully raised

# Measured lift positions reported on /joint_states (0..1), matching the
# normalised LIFT_DOWN/LIFT_UP targets above: 0.0 = retracted/down, 1.0 = extended/up.
LIFT_DOWN_POS    =  0.0
LIFT_UP_POS      =  1.0
LIFT_REACHED_TOL =  0.06  # treat the drum as "at" a target within this band

# These act as timeout fallbacks: transitions normally fire on the measured
# position above, but if feedback is missing the FSM still advances after this.
LOWER_DURATION_S =  3.0   # seconds to wait for drum to reach down position
RAISE_DURATION_S =  3.0   # seconds to wait for drum to reach up position

# How many times to re-issue a failed/aborted nav goal before giving up and
# holding safe (drum stopped) rather than digging at the wrong spot.
MAX_NAV_RETRIES  =  3

# Linear-actuator joints whose positions we watch on /joint_states.
LIFT_JOINTS = ("left_linear_actuator_joint", "right_linear_actuator_joint")

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

        # Measured lift position per actuator joint, populated from /joint_states.
        self._lift_positions = {}
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        self._nav = NavigationManager(self)
        self._nav_retry = 0

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

    def _nav_done(self, waypoint) -> bool:
        """True only once navigation to `waypoint` SUCCEEDED (FSM may advance).
        On a failed/aborted/rejected/timed-out goal, re-issues it up to
        MAX_NAV_RETRIES times, then stops the drum and enters HOLD instead of
        digging at the wrong spot. Returns False while navigating/retrying/held."""
        if not self._nav.goal_complete:
            return False
        if self._nav.goal_succeeded:
            self._nav_retry = 0
            return True
        self._nav_retry += 1
        if self._nav_retry <= MAX_NAV_RETRIES:
            self.get_logger().warn(
                f"Navigation failed — retry {self._nav_retry}/{MAX_NAV_RETRIES}"
            )
            self._nav.navigate_to(waypoint)
        else:
            self.get_logger().error("Navigation failed after retries — holding (drum stopped)")
            self._spin(0.0)
            self._enter("HOLD")
        return False

    # ── FSM ───────────────────────────────────────────────────────────────────

    def _loop(self):

        if self._state == "NAVIGATE_TO_START":
            self._nav.navigate_to(self._start)
            self._enter("WAIT_START")

        elif self._state == "WAIT_START":
            self._lift(LIFT_UP)  # hold up while navigating
            if self._nav_done(self._start):
                self.get_logger().info("Reached excavation start — spinning up")
                self._spin(DRUM_SPIN_SPEED)
                self._enter("LOWERING")

        elif self._state == "LOWERING":
            self._lift(LIFT_DOWN)  # command fully down (servo stops at the end-stop)
            reached = self._lift_reached(LIFT_DOWN_POS)
            if reached or self._elapsed() >= LOWER_DURATION_S:
                why = "drum down" if reached else "lower timeout"
                self.get_logger().info(f"Starting dig pass ({why})")
                self._nav.navigate_to(self._finish)
                self._enter("EXCAVATING")

        elif self._state == "EXCAVATING":
            self._lift(LIFT_DOWN)  # hold fully down while driving
            if self._nav_done(self._finish):
                self.get_logger().info("Reached excavation finish — stopping drum")
                self._spin(0.0)
                self._enter("RAISING")

        elif self._state == "RAISING":
            self._lift(LIFT_UP)  # command fully up (servo stops at the end-stop)
            if self._lift_reached(LIFT_UP_POS) or self._elapsed() >= RAISE_DURATION_S:
                self._enter("DONE")

        elif self._state == "DONE":
            self._lift(LIFT_UP)  # hold fully up
            self.get_logger().info("Excavation complete")
            self._timer.cancel()

        elif self._state == "HOLD":
            # Navigation failed past its retries — stay put with the drum stopped.
            # Nav2's cancel + the drive mux timeout keep the wheels stopped.
            self._spin(0.0)
            self.get_logger().error(
                "Excavation aborted — holding for operator intervention",
                throttle_duration_sec=5.0,
            )


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ExcavationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
