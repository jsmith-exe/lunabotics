import rclpy

from qpl_autonomy.autonomy_node import AutonomyNode
from qpl_autonomy.timed_sequences import TimedExcavationSequence, TimedDepositionSequence


EXCAVATION_KEYS = [
    "drive_speed",
    "drive_distance_m",
    "pulse_drive_s",
    "pulse_delay_s",
    "drum_spin_speed",
    "drum_spin_direction",
    "contact_position_m",
    "max_excavation_position_m",
    "raised_position_m",
    "position_tolerance_m",
    "drive_duration_s",
]

DEPOSITION_KEYS = [
    "drive_to_zone_speed",
    "drive_to_zone_distance_m",
    "deposit_drive_speed",
    "deposit_drive_distance_m",
    "max_height_position_m",
    "normal_height_position_m",
    "drum_spin_speed",
    "drum_spin_direction",
    "position_tolerance_m",
    "drive_to_zone_duration_s",
    "deposit_drive_duration_s",
]


class TimedAutonomyNode(AutonomyNode):
    """Blind autonomy where drive stages are ended by elapsed time instead of odometry.

    The node clock is fed to the sequences in place of the odometry x position,
    so the base FSMs record stage start times exactly where they would record
    start positions.
    """

    def __init__(self):
        super().__init__()

        # Parameters
        self.declare_parameter("excavation.drive_duration_s", 5.0)
        self.declare_parameter("deposition.drive_to_zone_duration_s", 7.5)
        self.declare_parameter("deposition.deposit_drive_duration_s", 20.0)
        excavation_config = {
            key: self.get_parameter(f"excavation.{key}").value for key in EXCAVATION_KEYS
        }
        deposition_config = {
            key: self.get_parameter(f"deposition.{key}").value for key in DEPOSITION_KEYS
        }

        # Sequences
        self.excavation = TimedExcavationSequence(
            cmd_vel_pub=self.cmd_vel_pub,
            drum_lift_pub=self.drum_lift_pub,
            drum_spin_pub=self.drum_spin_pub,
            config=excavation_config,
            logger=self.get_logger(),
        )
        self.deposition = TimedDepositionSequence(
            cmd_vel_pub=self.cmd_vel_pub,
            drum_lift_pub=self.drum_lift_pub,
            drum_spin_pub=self.drum_spin_pub,
            config=deposition_config,
            logger=self.get_logger(),
        )

        self.get_logger().info("Drive stages are timed.")

    """Feedback"""
    def odom_callback(self, msg):
        # Drivetrain odometry is not used
        pass

    def refresh_time(self):
        # Store the current time as the "position" passed to the sequences
        self.odom_x = self.get_clock().now().nanoseconds * 1e-9
        self.odom_y = 0.0

    """Basestation commands"""
    def handle_excavate(self):
        self.refresh_time()
        super().handle_excavate()

    def handle_deposit(self):
        self.refresh_time()
        super().handle_deposit()

    """FSM update"""
    def update_sequence(self):
        self.refresh_time()
        super().update_sequence()


def main(args=None):
    rclpy.init(args=args)

    node = TimedAutonomyNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
