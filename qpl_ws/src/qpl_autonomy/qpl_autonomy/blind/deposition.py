from enum import Enum, auto
from math import hypot

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class DepositionState(Enum):
    DRIVE_TO_CONSTRUCTION_ZONE = auto()
    LIFT_BUCKET = auto()
    DEPOSIT_REGOLITH = auto()
    STOP_WHEELS = auto()
    STOP_BUCKET = auto()
    RETURN_TO_NORMAL_HEIGHT = auto()
    COMPLETE = auto()


class DepositionSequence:
    """Feedback-based FSM for the competition deposition sequence."""

    # ---------------------------------------------------------
    # Calibration values
    # ---------------------------------------------------------

    DRIVE_SPEED_TO_ZONE = 0.2
    DEPOSIT_DRIVE_SPEED = 0.1

    DRIVE_TO_ZONE_DISTANCE_M = 1.5
    DEPOSIT_DRIVE_DISTANCE_M = 2.0

    # Tune these during mechanical testing.
    MAX_HEIGHT_POSITION_M = 0.20
    NORMAL_HEIGHT_POSITION_M = 0.00

    DRUM_SPIN_SPEED = 0.4

    # Change to -1.0 if the physical drum spins the wrong way.
    DRUM_SPIN_DIRECTION = 1.0

    POSITION_TOLERANCE_M = 0.005

    def __init__(
        self,
        cmd_vel_pub,
        drum_lift_pub,
        drum_spin_pub,
    ):
        self.cmd_vel_pub = cmd_vel_pub
        self.drum_lift_pub = drum_lift_pub
        self.drum_spin_pub = drum_spin_pub

        self.state = DepositionState.COMPLETE
        self.complete = False

        self.start_x = None
        self.start_y = None

    # =============================================================
    # Sequence lifecycle
    # =============================================================

    def start(self, actuator_positions, odom_position):
        """Start a new deposition sequence."""

        self.complete = False
        self.state = DepositionState.DRIVE_TO_CONSTRUCTION_ZONE

        self.start_x = None
        self.start_y = None

        if odom_position[0] is not None and odom_position[1] is not None:
            self.start_x = odom_position[0]
            self.start_y = odom_position[1]

    def update(self, actuator_positions, odom_position):
        """Run one non-blocking update of the FSM."""

        if self.complete:
            return

        if self.state == DepositionState.DRIVE_TO_CONSTRUCTION_ZONE:
            self.drive_forward(self.DRIVE_SPEED_TO_ZONE)

            if self.has_travelled_distance(
                odom_position,
                self.DRIVE_TO_ZONE_DISTANCE_M,
            ):
                self.stop_wheels()

                self.start_x = odom_position[0]
                self.start_y = odom_position[1]

                self.state = DepositionState.LIFT_BUCKET

        elif self.state == DepositionState.LIFT_BUCKET:
            self.stop_wheels()
            self.command_lift(self.MAX_HEIGHT_POSITION_M)

            if self.actuators_at_position(
                actuator_positions,
                self.MAX_HEIGHT_POSITION_M,
            ):
                self.start_x = odom_position[0]
                self.start_y = odom_position[1]

                self.state = DepositionState.DEPOSIT_REGOLITH

        elif self.state == DepositionState.DEPOSIT_REGOLITH:
            self.spin_bucket()
            self.drive_forward(self.DEPOSIT_DRIVE_SPEED)

            if self.has_travelled_distance(
                odom_position,
                self.DEPOSIT_DRIVE_DISTANCE_M,
            ):
                self.state = DepositionState.STOP_WHEELS

        elif self.state == DepositionState.STOP_WHEELS:
            self.stop_wheels()
            self.state = DepositionState.STOP_BUCKET

        elif self.state == DepositionState.STOP_BUCKET:
            self.stop_bucket()
            self.state = DepositionState.RETURN_TO_NORMAL_HEIGHT

        elif self.state == DepositionState.RETURN_TO_NORMAL_HEIGHT:
            self.command_lift(self.NORMAL_HEIGHT_POSITION_M)

            if self.actuators_at_position(
                actuator_positions,
                self.NORMAL_HEIGHT_POSITION_M,
            ):
                self.state = DepositionState.COMPLETE
                self.complete = True

    # =============================================================
    # Outputs
    # =============================================================

    def drive_forward(self, speed):
        command = Twist()
        command.linear.x = speed
        command.angular.z = 0.0
        self.cmd_vel_pub.publish(command)

    def stop_wheels(self):
        command = Twist()
        command.linear.x = 0.0
        command.angular.z = 0.0
        self.cmd_vel_pub.publish(command)

    def spin_bucket(self):
        command = Float64()
        command.data = (
            self.DRUM_SPIN_DIRECTION * self.DRUM_SPIN_SPEED
        )
        self.drum_spin_pub.publish(command)

    def stop_bucket(self):
        command = Float64()
        command.data = 0.0
        self.drum_spin_pub.publish(command)

    def command_lift(self, position):
        command = Float64()
        command.data = position
        self.drum_lift_pub.publish(command)

    # =============================================================
    # Feedback
    # =============================================================

    def actuators_at_position(self, actuator_positions, target):
        left, right = actuator_positions

        if left is None or right is None:
            return False

        left_error = abs(left - target)
        right_error = abs(right - target)

        return (
            left_error <= self.POSITION_TOLERANCE_M
            and right_error <= self.POSITION_TOLERANCE_M
        )

    def has_travelled_distance(self, odom_position, target_distance):
        x, y = odom_position

        if (
            x is None
            or y is None
            or self.start_x is None
            or self.start_y is None
        ):
            return False

        distance = hypot(
            x - self.start_x,
            y - self.start_y,
        )

        return distance >= target_distance

    # =============================================================
    # Stop
    # =============================================================

    def stop(self, actuator_positions):
        """Stop all autonomous outputs and hold the lift position."""

        self.stop_wheels()
        self.stop_bucket()

        left, right = actuator_positions

        if left is not None and right is not None:
            lift_command = Float64()
            lift_command.data = left
            self.drum_lift_pub.publish(lift_command)

            lift_command = Float64()
            lift_command.data = right
            self.drum_lift_pub.publish(lift_command)

        self.complete = False
        self.state = DepositionState.COMPLETE

    def is_complete(self):
        return self.complete