#!/usr/bin/env python3
"""
Blind excavation pass — no Nav2, no localization.

Presumes the rover is already standing at the dig start with a clean straight
path ahead. It lowers the drum, then creeps straight forward while spinning the
drum forward for a fixed time, then raises the drum. Movement is hardcoded
(open-loop Twist), not navigated.

It reuses the tuned drum/lift parameters from excavation_node so there is a
single source of truth, and drives the same drum/lift command topics. Forward
velocity goes to the drive mux's nav input (cmd_vel_nav) so teleop (higher
priority) can still override as an e-stop and the mux zeroes the output the
moment we stop publishing.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# Single source of truth for the tuned drum/lift parameters.
from qpl_autonomy.excavation_node import (
    DRUM_SPIN_SPEED,
    LIFT_DOWN,
    LIFT_UP,
    LIFT_DOWN_POS,
    LIFT_UP_POS,
    LIFT_REACHED_TOL,
    LIFT_JOINTS,
)


class BlindExcavationNode(Node):

    def __init__(self):
        super().__init__("blind_excavation_node")

        self.declare_parameter("forward_speed", 0.2)   # m/s, slow forward creep
        self.declare_parameter("drive_duration", 10.0)  # seconds of digging drive
        self._forward_speed = self.get_parameter(
            "forward_speed"
        ).get_parameter_value().double_value
        self._drive_duration = self.get_parameter(
            "drive_duration"
        ).get_parameter_value().double_value

        self._drum_pub = self.create_publisher(
            Float64, "/drum_spin_control/autonomy", 10
        )
        self._lift_pub = self.create_publisher(
            Float64, "/drum_lift_control/autonomy", 10
        )
        # Drive mux nav input — teleop (priority 100) can override; mux zeroes
        # the wheels when we stop publishing (0.25 s timeout).
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)

        # Measured lift position per actuator joint, populated from /joint_states.
        self._lift_positions = {}
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        self._state = "LOWERING"
        # Set on the first timer tick, not here: under use_sim_time get_clock().now()
        # is 0 until the first /clock message, so capturing it in __init__ makes the
        # first _elapsed() jump to a huge value and skip straight through the FSM.
        self._state_entry = None
        self._warned_waiting_clock = False

        self._timer = self.create_timer(0.1, self._loop)
        self.get_logger().info(
            "Blind excavation node started — presuming we are at the dig start"
        )

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

    def _drive(self, linear_x: float):
        msg = Twist()
        msg.linear.x = linear_x
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._cmd_pub.publish(Twist())

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

        # Wait for a valid clock (non-zero under sim time) before timing anything,
        # then anchor the first state's start to it.
        if self._state_entry is None:
            now = self.get_clock().now()
            if now.nanoseconds == 0:
                if not self._warned_waiting_clock:
                    self.get_logger().warn(
                        "Waiting for a valid clock (no /clock yet). "
                        "On hardware, launch with use_sim_time:=false."
                    )
                    self._warned_waiting_clock = True
                return
            self._state_entry = now

        if self._state == "LOWERING":
            self._lift(LIFT_DOWN)        # keep driving down until feedback says fully retracted
            self._spin(DRUM_SPIN_SPEED)  # spin up before the dig pass
            self._stop()                 # stay put while lowering
            if self._lift_reached(LIFT_DOWN_POS):
                self.get_logger().info("Drum fully lowered — starting dig pass")
                self._enter("DIGGING")
            else:
                self.get_logger().info(
                    "Lowering drum — waiting for feedback to reach fully retracted",
                    throttle_duration_sec=2.0,
                )

        elif self._state == "DIGGING":
            self._lift(LIFT_DOWN)        # hold fully down while driving
            self._spin(DRUM_SPIN_SPEED)
            self._drive(self._forward_speed)
            if self._elapsed() >= self._drive_duration:
                self.get_logger().info("Dig pass complete — stopping drum")
                self._stop()
                self._spin(0.0)
                self._enter("RAISING")

        elif self._state == "RAISING":
            self._stop()
            self._lift(LIFT_UP)          # keep driving up until feedback says fully extended
            if self._lift_reached(LIFT_UP_POS):
                self.get_logger().info("Drum fully raised")
                self._enter("DONE")
            else:
                self.get_logger().info(
                    "Raising drum — waiting for feedback to reach fully extended",
                    throttle_duration_sec=2.0,
                )

        elif self._state == "DONE":
            self._stop()
            self._lift(LIFT_UP)          # hold fully up
            self._spin(0.0)
            self.get_logger().info("Blind excavation complete")
            self._timer.cancel()


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = BlindExcavationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
