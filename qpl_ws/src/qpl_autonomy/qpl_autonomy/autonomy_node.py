from math import hypot

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from qpl_autonomy.excavation import ExcavationSequence
from qpl_autonomy.deposition import DepositionSequence
from qpl_autonomy.navigation import Navigation


class AutonomyNode(Node):
    """Main controller for autonomy for US competition.

    EXCAVATE / DEPOSIT run the blind sequences from where the rover is.
    NAV_EXCAVATE / NAV_DEPOSIT first drive to a map-frame waypoint with Nav2, then run
    the same sequence. AUTO repeats NAV_EXCAVATE then NAV_DEPOSIT until STOP.
    Nav2 is only needed for the NAV_ commands and AUTO.
    """

    def __init__(self):
        super().__init__("autonomy")

        # Parameters
        self.declare_parameter("autonomy.update_rate_hz", 5.0)
        update_rate_hz = self.get_parameter("autonomy.update_rate_hz").value

        self.declare_parameter("excavation.drum_spin_speed", 0.8)
        self.declare_parameter("excavation.drum_spin_direction", 1.0)
        self.declare_parameter("excavation.lowered_position_m", 0.03)
        self.declare_parameter("excavation.dig_duration_s", 30.0)
        self.declare_parameter("excavation.raised_position_m", 0.20)
        self.declare_parameter("excavation.position_tolerance_m", 0.005)
        excavation_config = {
            "drum_spin_speed": self.get_parameter("excavation.drum_spin_speed").value,
            "drum_spin_direction": self.get_parameter("excavation.drum_spin_direction").value,
            "lowered_position_m": self.get_parameter("excavation.lowered_position_m").value,
            "dig_duration_s": self.get_parameter("excavation.dig_duration_s").value,
            "raised_position_m": self.get_parameter("excavation.raised_position_m").value,
            "position_tolerance_m": self.get_parameter("excavation.position_tolerance_m").value,
        }

        self.declare_parameter("deposition.drive_to_zone_speed", 0.2)
        self.declare_parameter("deposition.drive_to_zone_distance_m", 1.5)
        self.declare_parameter("deposition.raised_position_m", 0.20)
        self.declare_parameter("deposition.dump_duration_s", 30.0)
        self.declare_parameter("deposition.drum_spin_speed", 0.4)
        self.declare_parameter("deposition.drum_spin_direction", -1.0)
        self.declare_parameter("deposition.position_tolerance_m", 0.005)
        deposition_config = {
            "drive_to_zone_speed": self.get_parameter("deposition.drive_to_zone_speed").value,
            "drive_to_zone_distance_m": self.get_parameter("deposition.drive_to_zone_distance_m").value,
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
        self.declare_parameter("navigation.travel_lift_position_m", 0.20)
        self.travel_lift_position_m = self.get_parameter("navigation.travel_lift_position_m").value
        # Waypoints are approximate: within this distance the sequence starts, whatever the
        # heading. Nav2 alone kept shuffling on the spot trying to line up exactly.
        self.declare_parameter("navigation.arrival_radius_m", 0.4)
        self.arrival_radius_m = self.get_parameter("navigation.arrival_radius_m").value

        # Start AUTO by itself once Nav2 is up and the rover has seen the AprilTag (qpl_auto_sim)
        self.declare_parameter("autonomy.auto_start", False)
        self.auto_start_pending = self.get_parameter("autonomy.auto_start").value
        self.tag_fix_received = False
        self.waypoints = {
            "EXCAVATION": self.get_parameter("navigation.excavation_waypoint").value,
            "DEPOSITION": self.get_parameter("navigation.deposition_waypoint").value,
        }
        self.max_nav_retries = self.get_parameter("navigation.max_retries").value
        self.nav_retries = 0
        self.navigation = Navigation(self, self.get_parameter("navigation.goal_timeout_s").value)

        # Top-level autonomy state
        self.state = "IDLE"

        # AUTO: after each sequence completes, navigate to the other zone instead of idling
        self.auto_mode = False
        self.auto_cycle = 0

        # Command publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.drum_lift_pub = self.create_publisher(Float64, "/drum_lift_control/autonomy", 10)
        self.drum_spin_pub = self.create_publisher(Float64, "/drum_spin_control/autonomy", 10)

        # Feedback
        self.left_actuator_position = None
        self.right_actuator_position = None
        self.odom_x = None
        self.odom_y = None
        self.joint_state_subscription = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, "/diff_cont/odom", self.odom_callback, 10)
        # Map-frame pose from the global EKF, for the arrival radius
        self.map_x = None
        self.map_y = None
        self.global_pose_subscription = self.create_subscription(Odometry, "/odometry/global", self.global_pose_callback, 10)
        self.tag_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/apriltag/pose_gated", self.tag_callback, 10)

        # Sequences
        self.excavation = ExcavationSequence(
            cmd_vel_pub=self.cmd_vel_pub,
            drum_lift_pub=self.drum_lift_pub,
            drum_spin_pub=self.drum_spin_pub,
            config=excavation_config,
        )
        self.deposition = DepositionSequence(
            cmd_vel_pub=self.cmd_vel_pub,
            drum_lift_pub=self.drum_lift_pub,
            drum_spin_pub=self.drum_spin_pub,
            config=deposition_config,
        )

        # Basestation command input
        self.command_subscription = self.create_subscription(
            String,
            "/autonomy/command",
            self.command_callback,
            10,
        )

        # Update the active FSM and refresh active commands at the configured rate
        self.sequence_timer = self.create_timer(
            1.0 / update_rate_hz,
            self.update_sequence,
        )

        self.get_logger().info("Autonomy started.")
        self.get_logger().info("State: IDLE")
        self.get_logger().info("Waiting for EXCAVATE, DEPOSIT, NAV_EXCAVATE, NAV_DEPOSIT, AUTO, or STOP command.")

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

    def tag_callback(self, msg):
        self.tag_fix_received = True

    def odom_callback(self, msg):
        #Store the latest drivetrain odometry position
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    """Basestation commands"""
    def command_callback(self, msg):
        # Handle commands received from the basestation
        command = msg.data.strip().upper()
        self.get_logger().info(f"Received autonomy command: {command}")

        if command == "EXCAVATE":
            self.handle_excavate()
        elif command == "DEPOSIT":
            self.handle_deposit()
        elif command == "NAV_EXCAVATE":
            self.handle_navigate("EXCAVATION")
        elif command == "NAV_DEPOSIT":
            self.handle_navigate("DEPOSITION")
        elif command == "AUTO":
            self.handle_auto()
        elif command == "STOP":
            self.handle_stop()
        else:
            self.get_logger().warning(f"Unknown autonomy command: {command}")

    def handle_excavate(self):
        # Start the excavation sequence
        if self.state != "IDLE":
            self.get_logger().warning(f"Cannot start excavation while in state: {self.state}")
            return

        self.state = "EXCAVATING"
        self.excavation.start()

        self.get_logger().info("Starting excavation sequence.")

    def handle_deposit(self):
        # Start the deposition sequence
        if self.state != "IDLE":
            self.get_logger().warning(f"Cannot start deposition while in state: {self.state}")
            return

        self.state = "DEPOSITING"
        self.deposition.start(odom_position=(self.odom_x, self.odom_y))

        self.get_logger().info("Starting deposition sequence.")

    def handle_navigate(self, target):
        # Drive to the target's waypoint with Nav2, then start its sequence
        if self.state != "IDLE":
            self.get_logger().warning(f"Cannot start navigation while in state: {self.state}")
            return
        if not self.navigation.is_available():
            self.get_logger().error("Nav2 is not running (start it with qpl_nav). Not starting.")
            return

        self.start_navigation(target)

    def handle_auto(self):
        # Loop excavation and deposition at their waypoints until STOP
        if self.state != "IDLE":
            self.get_logger().warning(f"Cannot start AUTO while in state: {self.state}")
            return
        if not self.navigation.is_available():
            self.get_logger().error("Nav2 is not running (start it with qpl_nav). Not starting.")
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
        x, y, yaw = self.waypoints[target]
        self.get_logger().info(f"Navigating to {target.lower()} waypoint ({x:.2f}, {y:.2f}, yaw {yaw:.2f}).")
        self.navigation.navigate_to(x, y, yaw)

    def handle_stop(self):
        # Stop the active autonomy sequence in its current position
        self.auto_start_pending = False
        if self.state == "IDLE":
            self.get_logger().info("Autonomy already idle.")
            return

        self.get_logger().info(f"Stopping autonomy from state: {self.state}")
        self.auto_mode = False
        actuator_positions=(self.left_actuator_position, self.right_actuator_position)

        if self.state == "EXCAVATING":
            self.excavation.stop(actuator_positions)
        elif self.state == "DEPOSITING":
            self.deposition.stop(actuator_positions)
        elif self.state.startswith("NAVIGATING_TO_"):
            # Nav2 keeps driving its goal unless it's cancelled
            self.navigation.cancel()

        self.state = "IDLE"
        self.get_logger().info("Autonomy stopped. State: IDLE")

    """FSM update"""
    def update_sequence(self):
        # Update whichever autonomy sequence is currently active
        actuator_positions = (self.left_actuator_position, self.right_actuator_position)
        odom_position = (self.odom_x, self.odom_y)

        if self.auto_start_pending and self.state == "IDLE":
            self.try_auto_start()

        if self.state.startswith("NAVIGATING_TO_"):
            self.update_navigation(self.state.removeprefix("NAVIGATING_TO_"), odom_position)
        elif self.state == "EXCAVATING":
            now_s = self.get_clock().now().nanoseconds / 1e9
            self.excavation.update(actuator_positions=actuator_positions, now_s=now_s)
            if self.excavation.is_complete():
                if self.auto_mode:
                    self.get_logger().info("Excavation complete.")
                    self.start_navigation("DEPOSITION")
                else:
                    self.state = "IDLE"
                    self.get_logger().info("Excavation complete. State: IDLE")
        elif self.state == "DEPOSITING":
            now_s = self.get_clock().now().nanoseconds / 1e9
            self.deposition.update(actuator_positions=actuator_positions, odom_position=odom_position, now_s=now_s)
            if self.deposition.is_complete():
                if self.auto_mode:
                    self.auto_cycle += 1
                    self.get_logger().info(f"Deposition complete. AUTO cycle {self.auto_cycle}.")
                    self.start_navigation("EXCAVATION")
                else:
                    self.state = "IDLE"
                    self.get_logger().info("Deposition complete. State: IDLE")

    def update_navigation(self, target, odom_position):
        # Wait for Nav2 to reach the waypoint, retrying a failed goal before giving up.
        # Hold the bucket raised so it doesn't plough the regolith on the way.
        self.drum_lift_pub.publish(Float64(data=self.travel_lift_position_m))
        self.navigation.update()

        # Close enough counts, even if Nav2 is still lining up or has given up on the spot
        distance = self.distance_to_waypoint(target)
        if distance is not None and distance <= self.arrival_radius_m and (
                not self.navigation.done or not self.navigation.succeeded):
            self.navigation.cancel()
            self.get_logger().info(
                f"Within {distance:.2f} m of {target.lower()} waypoint. Starting here.")
            self.start_sequence(target, odom_position)
            return

        if not self.navigation.done:
            return

        if self.navigation.succeeded:
            self.get_logger().info(f"Reached {target.lower()} waypoint.")
            self.start_sequence(target, odom_position)
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

    def distance_to_waypoint(self, target):
        if self.map_x is None or self.map_y is None:
            return None
        x, y, _ = self.waypoints[target]
        return hypot(self.map_x - x, self.map_y - y)

    def start_sequence(self, target, odom_position):
        if target == "EXCAVATION":
            self.state = "EXCAVATING"
            self.excavation.start()
            self.get_logger().info("Starting excavation sequence.")
        else:
            self.state = "DEPOSITING"
            # Navigation has already put the rover at the berm
            self.deposition.start(odom_position=odom_position, drive_to_zone=False)
            self.get_logger().info("Starting deposition sequence.")

    def try_auto_start(self):
        # autonomy.auto_start: begin AUTO once, as soon as Nav2 is up and the rover is localised
        if not self.navigation.is_available() or not self.tag_fix_received:
            self.get_logger().info(
                "Auto start: waiting for Nav2 and an AprilTag fix.", throttle_duration_sec=10.0)
            return
        self.auto_start_pending = False
        self.handle_auto()

    def shutdown(self):
        # Without this Nav2 keeps driving the last goal after the node dies
        if self.state.startswith("NAVIGATING_TO_"):
            self.get_logger().info("Shutting down. Cancelling navigation goal.")
            self.navigation.cancel()


def main(args=None):
    # Python's own Ctrl-C handling, so the context is still up to send the Nav2 cancel
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = AutonomyNode()

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