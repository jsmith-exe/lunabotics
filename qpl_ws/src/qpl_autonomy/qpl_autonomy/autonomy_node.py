import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
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
    the same sequence. Nav2 is only needed for the NAV_ commands.
    """

    def __init__(self):
        super().__init__("autonomy")

        # Parameters
        self.declare_parameter("autonomy.update_rate_hz", 5.0)
        update_rate_hz = self.get_parameter("autonomy.update_rate_hz").value

        self.declare_parameter("excavation.drive_speed", 0.2)
        self.declare_parameter("excavation.drive_distance_m", 1.0)
        self.declare_parameter("excavation.drum_spin_speed", 0.8)
        self.declare_parameter("excavation.drum_spin_direction", 1.0)
        self.declare_parameter("excavation.contact_position_m", 0.01)
        self.declare_parameter("excavation.max_excavation_position_m", 0.20)
        self.declare_parameter("excavation.raised_position_m", 0.00)
        self.declare_parameter("excavation.position_tolerance_m", 0.005)
        excavation_config = {
            "drive_speed": self.get_parameter("excavation.drive_speed").value,
            "drive_distance_m": self.get_parameter("excavation.drive_distance_m").value,
            "drum_spin_speed": self.get_parameter("excavation.drum_spin_speed").value,
            "drum_spin_direction": self.get_parameter("excavation.drum_spin_direction").value,
            "contact_position_m": self.get_parameter("excavation.contact_position_m").value,
            "max_excavation_position_m": self.get_parameter("excavation.max_excavation_position_m").value,
            "raised_position_m": self.get_parameter("excavation.raised_position_m").value,
            "position_tolerance_m": self.get_parameter("excavation.position_tolerance_m").value,
        }

        self.declare_parameter("deposition.drive_to_zone_speed", 0.2)
        self.declare_parameter("deposition.drive_to_zone_distance_m", 1.5)
        self.declare_parameter("deposition.deposit_drive_speed", 0.1)
        self.declare_parameter("deposition.deposit_drive_distance_m", 2.0)
        self.declare_parameter("deposition.max_height_position_m", 0.20)
        self.declare_parameter("deposition.normal_height_position_m", 0.00,)
        self.declare_parameter("deposition.drum_spin_speed", 0.4)
        self.declare_parameter("deposition.drum_spin_direction", 1.0)
        self.declare_parameter("deposition.position_tolerance_m", 0.005)
        deposition_config = {
            "drive_to_zone_speed": self.get_parameter("deposition.drive_to_zone_speed").value,
            "drive_to_zone_distance_m": self.get_parameter("deposition.drive_to_zone_distance_m").value,
            "deposit_drive_speed": self.get_parameter("deposition.deposit_drive_speed").value,
            "deposit_drive_distance_m": self.get_parameter("deposition.deposit_drive_distance_m").value,
            "max_height_position_m": self.get_parameter("deposition.max_height_position_m").value,
            "normal_height_position_m": self.get_parameter("deposition.normal_height_position_m").value,
            "drum_spin_speed": self.get_parameter("deposition.drum_spin_speed").value,
            "drum_spin_direction": self.get_parameter("deposition.drum_spin_direction").value,
            "position_tolerance_m": self.get_parameter("deposition.position_tolerance_m").value,
        }

        # Waypoints, map frame: [x, y, yaw]. Each is where its sequence starts, facing the
        # direction it drives in.
        self.declare_parameter("navigation.excavation_waypoint", [6.0, 1.5, 0.0])
        self.declare_parameter("navigation.deposition_waypoint", [8.14, 4.9, 1.5708])
        self.declare_parameter("navigation.goal_timeout_s", 180.0)
        self.declare_parameter("navigation.max_retries", 2)
        self.waypoints = {
            "EXCAVATION": self.get_parameter("navigation.excavation_waypoint").value,
            "DEPOSITION": self.get_parameter("navigation.deposition_waypoint").value,
        }
        self.max_nav_retries = self.get_parameter("navigation.max_retries").value
        self.nav_retries = 0
        self.navigation = Navigation(self, self.get_parameter("navigation.goal_timeout_s").value)

        # Top-level autonomy state
        self.state = "IDLE"

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
        self.get_logger().info("Waiting for EXCAVATE, DEPOSIT, NAV_EXCAVATE, NAV_DEPOSIT, or STOP command.")

    """Feedback callbacks"""
    def joint_state_callback(self, msg):
        # Store the latest linear actuator positions
        for name, position in zip(msg.name, msg.position):
            if name == "left_linear_actuator_joint":
                self.left_actuator_position = position
            elif name == "right_linear_actuator_joint":
                self.right_actuator_position = position

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
        self.excavation.start(odom_position=(self.odom_x, self.odom_y))

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

        self.state = f"NAVIGATING_TO_{target}"
        self.nav_retries = 0
        self.send_navigation_goal(target)

    def send_navigation_goal(self, target):
        x, y, yaw = self.waypoints[target]
        self.get_logger().info(f"Navigating to {target.lower()} waypoint ({x:.2f}, {y:.2f}, yaw {yaw:.2f}).")
        self.navigation.navigate_to(x, y, yaw)

    def handle_stop(self):
        # Stop the active autonomy sequence in its current position
        if self.state == "IDLE":
            self.get_logger().info("Autonomy already idle.")
            return

        self.get_logger().info(f"Stopping autonomy from state: {self.state}")
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

        if self.state.startswith("NAVIGATING_TO_"):
            self.update_navigation(self.state.removeprefix("NAVIGATING_TO_"), odom_position)
        elif self.state == "EXCAVATING":
            self.excavation.update(actuator_positions=actuator_positions, odom_position=odom_position)
            if self.excavation.is_complete():
                self.state = "IDLE"
                self.get_logger().info("Excavation complete. State: IDLE")
        elif self.state == "DEPOSITING":
            self.deposition.update(actuator_positions=actuator_positions, odom_position=odom_position)
            if self.deposition.is_complete():
                self.state = "IDLE"
                self.get_logger().info("Deposition complete. State: IDLE")

    def update_navigation(self, target, odom_position):
        # Wait for Nav2 to reach the waypoint, retrying a failed goal before giving up
        self.navigation.update()
        if not self.navigation.done:
            return

        if self.navigation.succeeded:
            self.get_logger().info(f"Reached {target.lower()} waypoint.")
            if target == "EXCAVATION":
                self.state = "EXCAVATING"
                self.excavation.start(odom_position=odom_position)
                self.get_logger().info("Starting excavation sequence.")
            else:
                self.state = "DEPOSITING"
                # Navigation has already put the rover at the deposit start
                self.deposition.start(odom_position=odom_position, drive_to_zone=False)
                self.get_logger().info("Starting deposition sequence.")
            return

        self.nav_retries += 1
        if self.nav_retries <= self.max_nav_retries:
            self.get_logger().warning(f"Navigation failed. Retry {self.nav_retries}/{self.max_nav_retries}.")
            self.send_navigation_goal(target)
        else:
            self.get_logger().error("Navigation failed after retries. State: IDLE")
            self.cmd_vel_pub.publish(Twist())
            self.state = "IDLE"

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