#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from qpl_autonomy.navigation_manager import NavigationManager

# ── Tunable parameters ────────────────────────────────────────────────────────
DRUM_SPIN_SPEED  =  5.0   # rad/s, forward spin
DRUM_LOWER_SPEED = -0.05  # m/s, downward
DRUM_RAISE_SPEED =  0.05  # m/s, upward

LOWER_DURATION_S =  3.0   # seconds to lower drum before driving
RAISE_DURATION_S =  3.0   # seconds to raise drum after arriving

EXCAVATION_START  = {"x": 1.7, "y": 3.0, "yaw": 0.0}
EXCAVATION_FINISH = {"x": 3.7, "y": 3.0, "yaw": 0.0}  # 2 m further on x
# ─────────────────────────────────────────────────────────────────────────────


class ExcavationNode(Node):

    def __init__(self):
        super().__init__("excavation_node")

        self._drum_pub = self.create_publisher(
            Float64, "/drum_cont/cmd_vel_unstamped", 10
        )
        self._lift_pub = self.create_publisher(
            Float64, "/drum_lift_cont/cmd_vel_unstamped", 10
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

    def _lift(self, speed: float):
        self._lift_pub.publish(Float64(data=speed))

    # ── FSM ───────────────────────────────────────────────────────────────────

    def _loop(self):

        if self._state == "NAVIGATE_TO_START":
            self._nav.navigate_to(EXCAVATION_START)
            self._enter("WAIT_START")

        elif self._state == "WAIT_START":
            self._lift(0.0)
            if self._nav.goal_complete:
                self.get_logger().info("Reached excavation start — spinning up")
                self._spin(DRUM_SPIN_SPEED)
                self._enter("LOWERING")

        elif self._state == "LOWERING":
            # Keep publishing the lower command every tick so the lift
            # doesn't stall on a missed message.
            self._lift(DRUM_LOWER_SPEED)
            if self._elapsed() >= LOWER_DURATION_S:
                # Explicitly stop the lift before driving.
                self._lift(0.0)
                self._nav.navigate_to(EXCAVATION_FINISH)
                self._enter("EXCAVATING")

        elif self._state == "EXCAVATING":
            # Hold lift at zero while driving — prevents erratic movement
            # if the joint is already at its lower limit.
            self._lift(0.0)
            if self._nav.goal_complete:
                self.get_logger().info("Reached excavation finish — stopping drum")
                self._spin(0.0)
                self._enter("RAISING")

        elif self._state == "RAISING":
            # Keep publishing the raise command every tick.
            self._lift(DRUM_RAISE_SPEED)
            if self._elapsed() >= RAISE_DURATION_S:
                # Explicitly stop the lift — prevents erratic behaviour at
                # the upper limit.
                self._lift(0.0)
                self._enter("DONE")

        elif self._state == "DONE":
            self._lift(0.0)
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
