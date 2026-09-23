#!/usr/bin/env python3
"""
Blind construction (berm-building) pass — no Nav2, no localization.

Presumes the rover is already standing with the drum loaded and a clean
straight path ahead. It holds the drum fully raised, creeps straight forward
for a fixed time, then reverse-spins the drum for a fixed time to dump the load
and build the berm.

It reuses the tuned drum parameters from excavation_node / deposition_node so
there is a single source of truth, and drives the same drum/lift command
topics. Forward velocity goes to the drive mux's nav input (cmd_vel_nav) so
teleop (higher priority) can still override as an e-stop and the mux zeroes the
output the moment we stop publishing.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# Single source of truth for the tuned drum/lift parameters.
from qpl_autonomy.excavation_node import LIFT_UP
from qpl_autonomy.deposition_node import DRUM_SPIN_SPEED as DUMP_SPIN  # reverse spin while dumping


class BlindConstructionNode(Node):

    def __init__(self):
        super().__init__("blind_construction_node")

        self.declare_parameter("forward_speed", 0.2)    # m/s, slow forward creep
        self.declare_parameter("drive_duration", 10.0)  # seconds of forward drive
        self.declare_parameter("dump_duration", 10.0)   # seconds of reverse spin
        self._forward_speed = self.get_parameter(
            "forward_speed"
        ).get_parameter_value().double_value
        self._drive_duration = self.get_parameter(
            "drive_duration"
        ).get_parameter_value().double_value
        self._dump_duration = self.get_parameter(
            "dump_duration"
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

        self._state = "DRIVING"
        # Set on the first timer tick, not here: under use_sim_time get_clock().now()
        # is 0 until the first /clock message, so capturing it in __init__ makes the
        # first _elapsed() jump to a huge value and finish the dump instantly.
        self._state_entry = None
        self._warned_waiting_clock = False

        self._timer = self.create_timer(0.1, self._loop)
        self.get_logger().info(
            "Blind construction node started — reverse-spinning to build the berm"
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

        if self._state == "DRIVING":
            self._lift(LIFT_UP)              # hold fully up the whole time
            self._drive(self._forward_speed)  # creep straight forward to the berm
            if self._elapsed() >= self._drive_duration:
                self.get_logger().info("Forward drive complete — stopping to dump")
                self._stop()
                self._enter("DEPOSITING")

        elif self._state == "DEPOSITING":
            self._stop()             # stay put while dumping
            self._lift(LIFT_UP)      # hold fully up the whole time
            self._spin(DUMP_SPIN)    # reverse spin to dump / build berm
            if self._elapsed() >= self._dump_duration:
                self.get_logger().info("Dump complete — stopping drum")
                self._spin(0.0)
                self._enter("DONE")

        elif self._state == "DONE":
            self._stop()
            self._spin(0.0)
            self._lift(LIFT_UP)      # leave drum raised
            self.get_logger().info("Blind construction complete")
            self._timer.cancel()


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = BlindConstructionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
