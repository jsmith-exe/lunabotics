import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from qpl_autonomy.excavation import ExcavationSequence
from qpl_autonomy.deposition import DepositionSequence


class AutonomyNode(Node):
    """Main controller for blind autonomy for US competition."""

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
        self.get_logger().info("Waiting for EXCAVATE, DEPOSIT, or STOP command.")

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

        self.state = "IDLE"
        self.get_logger().info("Autonomy stopped. State: IDLE")

    """FSM update"""
    def update_sequence(self):
        # Update whichever autonomy sequence is currently active
        actuator_positions = (self.left_actuator_position, self.right_actuator_position)
        odom_position = (self.odom_x, self.odom_y)

        if self.state == "EXCAVATING":
            self.excavation.update(actuator_positions=actuator_positions, odom_position=odom_position)
            if self.excavation.is_complete():
                self.state = "IDLE"
                self.get_logger().info("Excavation complete. State: IDLE")
        elif self.state == "DEPOSITING":
            self.deposition.update(actuator_positions=actuator_positions, odom_position=odom_position)
            if self.deposition.is_complete():
                self.state = "IDLE"
                self.get_logger().info("Deposition complete. State: IDLE")


def main(args=None):
    rclpy.init(args=args)

    node = AutonomyNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()