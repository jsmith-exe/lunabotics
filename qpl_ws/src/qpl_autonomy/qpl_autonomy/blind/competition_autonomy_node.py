import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from qpl_autonomy.blind.excavation import ExcavationSequence


class CompetitionAutonomyNode(Node):
    """Main controller for competition/blind autonomy."""

    def __init__(self):
        super().__init__("competition_autonomy")

        # Current autonomy state
        self.state = "IDLE"

        # Publisher used by autonomy to command the drivetrain
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel_nav",
            10,
        )

        # Excavation sequence
        self.excavation = ExcavationSequence(
            cmd_vel_pub=self.cmd_vel_pub,
        )

        # Receive commands from the basestation.
        self.command_subscription = self.create_subscription(
            String,
            "/autonomy/command",
            self.command_callback,
            10,
        )

        # Update the active autonomy sequence at 10 Hz
        self.sequence_timer = self.create_timer(
            0.1,
            self.update_sequence,
        )

        self.get_logger().info("Competition autonomy started.")
        self.get_logger().info("State: IDLE")
        self.get_logger().info("Waiting for EXCAVATE, DEPOSIT, or STOP command.")

    def command_callback(self, msg): # Handle commands received from the basestation
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

    def handle_excavate(self): # Start the excavation sequence
        if self.state != "IDLE":
            self.get_logger().warning(f"Cannot start excavation while in state: {self.state}")
            return

        self.state = "EXCAVATING"
        self.get_logger().info("Starting excavation sequence.")
        self.excavation.start()

    def handle_deposit(self): # Start the deposition sequence
        if self.state != "IDLE":
            self.get_logger().warning(f"Cannot start deposition while in state: {self.state}")
            return

        self.state = "DEPOSITING"
        self.get_logger().info("Starting deposition sequence.")
        # TODO:
        # Start the deposition sequence here

    def handle_stop(self): # Stop the current autonomy sequence
        if self.state == "IDLE":
            self.get_logger().info("Autonomy already idle.")
            return

        self.get_logger().info(f"Stopping autonomy from state: {self.state}")

        if self.state == "EXCAVATING":
            self.excavation.stop()

        self.state = "IDLE"
        self.get_logger().info("Autonomy stopped. State: IDLE")

    def update_sequence(self): # Update the currently active autonomy sequence
        if self.state == "EXCAVATING":
            self.excavation.update()

            if self.excavation.is_complete():
                self.state = "IDLE"

                self.get_logger().info(
                    "Excavation complete. State: IDLE"
                )

def main(args=None):
    rclpy.init(args=args)
    node = CompetitionAutonomyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()