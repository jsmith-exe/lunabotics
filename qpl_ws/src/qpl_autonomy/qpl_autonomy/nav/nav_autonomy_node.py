import time
from math import atan2, hypot, pi

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from qpl_autonomy.nav.nav_excavation import NavExcavationSequence
from qpl_autonomy.nav.nav_deposition import NavDepositionSequence
from qpl_autonomy.navigation import Navigation

# Seconds after startup before a missing /clock counts as an error
CLOCK_GRACE_S = 5.0


class NavAutonomyNode(Node):
    """Nav2 based autonomy for US competition.

    Separate from the blind autonomy node (autonomy_node and its variants), which is left
    as it is as a backup. Started with qpl_nav, but sits IDLE until a command arrives.

    NAV_EXCAVATE / NAV_DEPOSIT drive to a map-frame waypoint with Nav2, then dig or dump
    standing still. AUTO (full autonomy) repeats NAV_EXCAVATE then NAV_DEPOSIT until STOP.
    """

    def __init__(self):
        super().__init__("nav_autonomy")

        # Parameters
        self.declare_parameter("autonomy.update_rate_hz", 5.0)
        update_rate_hz = self.get_parameter("autonomy.update_rate_hz").value

        self.declare_parameter("excavation.drum_spin_speed", 0.8)
        self.declare_parameter("excavation.drum_spin_direction", 1.0)
        self.declare_parameter("excavation.lowered_position_m", 0.03)
        self.declare_parameter("excavation.dig_duration_s", 30.0)
        self.declare_parameter("excavation.raised_position_m", 0.22)
        self.declare_parameter("excavation.position_tolerance_m", 0.005)
        excavation_config = {
            "drum_spin_speed": self.get_parameter("excavation.drum_spin_speed").value,
            "drum_spin_direction": self.get_parameter("excavation.drum_spin_direction").value,
            "lowered_position_m": self.get_parameter("excavation.lowered_position_m").value,
            "dig_duration_s": self.get_parameter("excavation.dig_duration_s").value,
            "raised_position_m": self.get_parameter("excavation.raised_position_m").value,
            "position_tolerance_m": self.get_parameter("excavation.position_tolerance_m").value,
        }

        self.declare_parameter("deposition.raised_position_m", 0.22)
        self.declare_parameter("deposition.dump_duration_s", 30.0)
        self.declare_parameter("deposition.drum_spin_speed", 0.4)
        self.declare_parameter("deposition.drum_spin_direction", 1.0)
        self.declare_parameter("deposition.position_tolerance_m", 0.005)
        deposition_config = {
            "raised_position_m": self.get_parameter("deposition.raised_position_m").value,
            "dump_duration_s": self.get_parameter("deposition.dump_duration_s").value,
            "drum_spin_speed": self.get_parameter("deposition.drum_spin_speed").value,
            "drum_spin_direction": self.get_parameter("deposition.drum_spin_direction").value,
            "position_tolerance_m": self.get_parameter("deposition.position_tolerance_m").value,
        }

        # Waypoints, map frame: [x, y, yaw]. The rover stands still at each one to dig or dump.
        self.declare_parameter("navigation.excavation_waypoint", [6.0, 1.5, 0.0])
        self.declare_parameter("navigation.deposition_waypoint", [8.14, 6.40, 1.5708])
        self.declare_parameter("navigation.goal_timeout_s", 180.0)
        self.declare_parameter("navigation.max_retries", 2)
        self.declare_parameter("navigation.travel_lift_position_m", 0.22)
        self.travel_lift_position_m = self.get_parameter("navigation.travel_lift_position_m").value
        # Within this distance and heading of a waypoint the sequence starts, even if Nav2 is
        # still lining up. A small buffer so it doesn't keep shuffling to correct itself.
        self.declare_parameter("navigation.arrival_radius_m", 0.25)
        self.arrival_radius_m = self.get_parameter("navigation.arrival_radius_m").value
        self.declare_parameter("navigation.arrival_yaw_tolerance_rad", 0.3)
        self.arrival_yaw_tolerance_rad = self.get_parameter("navigation.arrival_yaw_tolerance_rad").value

        self.waypoints = {
            "EXCAVATION": self.get_parameter("navigation.excavation_waypoint").value,
            "DEPOSITION": self.get_parameter("navigation.deposition_waypoint").value,
        }
        self.max_nav_retries = self.get_parameter("navigation.max_retries").value
        self.nav_retries = 0
        self.navigation = Navigation(self, self.get_parameter("navigation.goal_timeout_s").value)
        # A goal waiting for Nav2 to finish cancelling the previous one (see Navigation.ready)
        self.goal_pending = False

        # Top-level autonomy state
        self.state = "IDLE"

        # AUTO: after each sequence completes, navigate to the other zone instead of idling
        self.auto_mode = False
        self.auto_cycle = 0

        # Command publishers, the same ones the blind autonomy node uses
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.drum_lift_pub = self.create_publisher(Float64, "/drum_lift_control/autonomy", 10)
        self.drum_spin_pub = self.create_publisher(Float64, "/drum_spin_control/autonomy", 10)

        # Feedback
        self.left_actuator_position = None
        self.right_actuator_position = None
        self.joint_state_subscription = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        # Map-frame pose from the global EKF, for the arrival radius
        self.map_x = None
        self.map_y = None
        self.map_yaw = None
        self.global_pose_subscription = self.create_subscription(Odometry, "/odometry/global", self.global_pose_callback, 10)

        # Sequences
        self.excavation = NavExcavationSequence(
            cmd_vel_pub=self.cmd_vel_pub,
            drum_lift_pub=self.drum_lift_pub,
            drum_spin_pub=self.drum_spin_pub,
            config=excavation_config,
        )
        self.deposition = NavDepositionSequence(
            cmd_vel_pub=self.cmd_vel_pub,
            drum_lift_pub=self.drum_lift_pub,
            drum_spin_pub=self.drum_spin_pub,
            config=deposition_config,
        )

        # Basestation command input, separate from the blind node's /autonomy/command
        self.command_subscription = self.create_subscription(
            String,
            "/nav_autonomy/command",
            self.command_callback,
            10,
        )

        # Update the active FSM and refresh active commands at the configured rate
        self.sequence_timer = self.create_timer(
            1.0 / update_rate_hz,
            self.update_sequence,
        )

        # The timer above runs on ROS time: with use_sim_time and no /clock (qpl_nav_sim on the
        # rover) it never fires, and nothing would say why. This one runs on the steady clock.
        self.started_s = time.monotonic()
        self.time_ok = True
        self.time_message = ""
        self.health_timer = self.create_timer(1.0, self.check_health, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info("Nav autonomy started.")
        self.get_logger().info("State: IDLE")
        self.get_logger().info("Waiting for NAV_EXCAVATE, NAV_DEPOSIT, AUTO, or STOP command.")

    """Feedback callbacks"""
    def joint_state_callback(self, msg):
        # Store the latest linear actuator positions
        for name, position in zip(msg.name, msg.position):
            if name == "left_linear_actuator_joint":
                self.left_actuator_position = position
            elif name == "right_linear_actuator_joint":
                self.right_actuator_position = position

    def global_pose_callback(self, msg):
        self.map_x = msg.pose.pose.position.x
        self.map_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.map_yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    """Basestation commands"""
    def command_callback(self, msg):
        command = msg.data.strip().upper()

        if command == "NAV_EXCAVATE":
            self.handle_navigate("EXCAVATION")
        elif command == "NAV_DEPOSIT":
            self.handle_navigate("DEPOSITION")
        elif command == "AUTO":
            self.handle_auto()
        elif command == "STOP":
            self.handle_stop()
        else:
            self.get_logger().warning(f"Unknown nav autonomy command: {command}")

    def can_start(self, command):
        # Whether a new command may start; logs why not
        if self.state != "IDLE":
            self.get_logger().warning(f"Cannot start {command} while in state: {self.state}")
            return False
        if not self.time_ok:
            self.get_logger().error(f"Not starting {command}: {self.time_message}")
            return False
        if not self.navigation.is_available():
            self.get_logger().error(f"Not starting {command}: Nav2 is not running (start it with qpl_nav, or qpl_nav_sim against the sim).")
            return False
        return True

    def handle_navigate(self, target):
        # Drive to the target's waypoint with Nav2, then start its sequence
        if not self.can_start(f"NAV_{target}"):
            return

        self.start_navigation(target)

    def handle_auto(self):
        # Loop excavation and deposition at their waypoints until STOP
        if not self.can_start("AUTO"):
            return

        self.auto_mode = True
        self.auto_cycle = 1
        self.get_logger().info("AUTO started. Cycle 1.")
        self.start_navigation("EXCAVATION")

    def start_navigation(self, target):
        self.state = f"NAVIGATING_TO_{target}"
        self.nav_retries = 0
        self.send_navigation_goal(target)

    def send_navigation_goal(self, target):
        # Sent while Nav2 is still cancelling the last goal, Nav2 aborts it; update_navigation
        # sends it once the cancel has finished
        if not self.navigation.ready():
            self.goal_pending = True
            return
        self.goal_pending = False
        x, y, yaw = self.waypoints[target]
        self.get_logger().info(f"Navigating to {target.lower()} waypoint ({x:.2f}, {y:.2f}, yaw {yaw:.2f}).")
        self.navigation.navigate_to(x, y, yaw)

    def handle_stop(self):
        # Stop the active sequence or navigation in its current position
        if self.state == "IDLE":
            self.get_logger().info("Nav autonomy already idle.")
            return

        self.get_logger().info(f"Stopping nav autonomy from state: {self.state}")
        self.auto_mode = False
        self.goal_pending = False
        actuator_positions = (self.left_actuator_position, self.right_actuator_position)

        if self.state == "EXCAVATING":
            self.excavation.stop(actuator_positions)
        elif self.state == "DEPOSITING":
            self.deposition.stop(actuator_positions)
        elif self.state.startswith("NAVIGATING_TO_"):
            # Nav2 keeps driving its goal unless it's cancelled
            self.navigation.cancel()
            self.cmd_vel_pub.publish(Twist())

        self.state = "IDLE"
        self.get_logger().info("Nav autonomy stopped. State: IDLE")

    """FSM update"""
    def update_sequence(self):
        actuator_positions = (self.left_actuator_position, self.right_actuator_position)
        now_s = self.get_clock().now().nanoseconds / 1e9

        if self.state.startswith("NAVIGATING_TO_"):
            self.update_navigation(self.state.removeprefix("NAVIGATING_TO_"))
        elif self.state == "EXCAVATING":
            self.excavation.update(actuator_positions=actuator_positions, now_s=now_s)
            if self.excavation.is_complete():
                if self.auto_mode:
                    self.get_logger().info("Excavation complete.")
                    self.start_navigation("DEPOSITION")
                else:
                    self.state = "IDLE"
                    self.get_logger().info("Excavation complete. State: IDLE")
        elif self.state == "DEPOSITING":
            self.deposition.update(actuator_positions=actuator_positions, now_s=now_s)
            if self.deposition.is_complete():
                if self.auto_mode:
                    self.auto_cycle += 1
                    self.get_logger().info(f"Deposition complete. AUTO cycle {self.auto_cycle}.")
                    self.start_navigation("EXCAVATION")
                else:
                    self.state = "IDLE"
                    self.get_logger().info("Deposition complete. State: IDLE")

    def update_navigation(self, target):
        # Wait for Nav2 to reach the waypoint, retrying a failed goal before giving up.
        # Hold the bucket raised so it doesn't plough the regolith on the way.
        self.drum_lift_pub.publish(Float64(data=self.travel_lift_position_m))
        if self.goal_pending:
            # The last goal is still being cancelled; done/succeeded below still describe it
            self.send_navigation_goal(target)
            return
        self.navigation.update()

        # Close enough and facing the right way counts, even if Nav2 is still lining up or
        # has given up on the spot
        error = self.waypoint_error(target)
        if error is not None and error[0] <= self.arrival_radius_m and error[1] <= self.arrival_yaw_tolerance_rad and (
                not self.navigation.done or not self.navigation.succeeded):
            self.navigation.cancel()
            self.get_logger().info(
                f"Within {error[0]:.2f} m and {error[1]:.2f} rad of {target.lower()} waypoint. Starting here.")
            self.start_sequence(target)
            return

        if not self.navigation.done:
            return

        if self.navigation.succeeded:
            if error is not None:
                self.get_logger().info(
                    f"Reached {target.lower()} waypoint ({error[0]:.2f} m, {error[1]:.2f} rad off).")
            else:
                self.get_logger().info(f"Reached {target.lower()} waypoint.")
            self.start_sequence(target)
            return

        self.nav_retries += 1
        if self.nav_retries <= self.max_nav_retries:
            self.get_logger().warning(f"Navigation failed. Retry {self.nav_retries}/{self.max_nav_retries}.")
            self.send_navigation_goal(target)
        else:
            self.get_logger().error("Navigation failed after retries. State: IDLE")
            self.cmd_vel_pub.publish(Twist())
            self.auto_mode = False
            self.state = "IDLE"

    def waypoint_error(self, target):
        # (distance, absolute heading error) from the waypoint, or None without a map pose
        if self.map_x is None or self.map_y is None or self.map_yaw is None:
            return None
        x, y, yaw = self.waypoints[target]
        yaw_error = abs((self.map_yaw - yaw + pi) % (2.0 * pi) - pi)
        return hypot(self.map_x - x, self.map_y - y), yaw_error

    def start_sequence(self, target):
        if target == "EXCAVATION":
            self.state = "EXCAVATING"
            self.excavation.start()
            self.get_logger().info("Starting excavation sequence.")
        else:
            self.state = "DEPOSITING"
            self.deposition.start()
            self.get_logger().info("Starting deposition sequence.")

    """Health (steady clock, so it runs even when ROS time is stuck)"""
    def check_clock(self):
        # (ok, message): is ROS time usable for this node's timers and Nav2's?
        if self.get_parameter("use_sim_time").value:
            if self.get_clock().now().nanoseconds > 0 or time.monotonic() - self.started_s < CLOCK_GRACE_S:
                return True, ""
            return False, ("use_sim_time is true but nothing publishes /clock. On the rover start Nav2 with "
                           "qpl_nav; qpl_nav_sim is for the sim.")
        if self.count_publishers("/clock") > 0:
            return False, "the sim is publishing /clock but this node runs on wall time. Start Nav2 with qpl_nav_sim."
        return True, ""

    def check_health(self):
        ok, message = self.check_clock()
        if (ok, message) != (self.time_ok, self.time_message):
            if ok:
                self.get_logger().info("Clock OK.")
            else:
                self.get_logger().error(f"Clock problem: {message}")
        self.time_ok, self.time_message = ok, message

    def shutdown(self):
        # Without this Nav2 keeps driving the last goal after the node dies
        if self.state.startswith("NAVIGATING_TO_"):
            self.get_logger().info("Shutting down. Cancelling navigation goal.")
            self.navigation.cancel()


def main(args=None):
    # Python's own Ctrl-C handling, so the context is still up to send the Nav2 cancel
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = NavAutonomyNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.shutdown()
        # Flush the cancel request before the node goes away
        rclpy.spin_once(node, timeout_sec=0.5)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
