from enum import Enum, auto
from math import hypot

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class ExcavationState(Enum):
    STOP_WHEELS = auto()
    LOWER_TO_CONTACT = auto()
    LOWER_TO_MAX = auto()
    DRIVE_FORWARD = auto()
    STOP_WHEELS_AFTER_DRIVE = auto()
    LIFT_BUCKET = auto()
    STOP_BUCKET = auto()
    COMPLETE = auto()


class ExcavationSequence:
    """Feedback-based FSM for the excavation sequence."""

    def __init__(
        self,
        cmd_vel_pub,
        drum_lift_pub,
        drum_spin_pub,
        config,
    ):
        self.cmd_vel_pub = cmd_vel_pub
        self.drum_lift_pub = drum_lift_pub
        self.drum_spin_pub = drum_spin_pub

        self.drive_speed = config["drive_speed"]
        self.drive_distance_m = config["drive_distance_m"]
        self.drum_spin_speed = config["drum_spin_speed"]
        self.drum_spin_direction = config["drum_spin_direction"]
        self.contact_position_m = config["contact_position_m"]
        self.max_excavation_position_m = config["max_excavation_position_m"]
        self.raised_position_m = config["raised_position_m"]
        self.position_tolerance_m = config["position_tolerance_m"]

        self.state = ExcavationState.COMPLETE
        self.complete = False

        self.start_x = None
        self.start_y = None

    """Sequence lifecycle"""

    def start(self, odom_position):
        # Start a new excavation sequence
        self.complete = False
        self.state = ExcavationState.STOP_WHEELS
        self.start_x = None
        self.start_y = None
        if odom_position[0] is not None and odom_position[1] is not None:
            self.start_x = odom_position[0]
            self.start_y = odom_position[1]

    def update(self, actuator_positions, odom_position):
        # Run one non-blocking update of the FSM
        if self.complete:
            return

        # 1. Stop wheels
        if self.state == ExcavationState.STOP_WHEELS:
            self.stop_wheels()
            self.state = ExcavationState.LOWER_TO_CONTACT

        # 2. Lower bucket to contact while spinning
        elif self.state == ExcavationState.LOWER_TO_CONTACT:
            self.spin_bucket()
            self.command_lift(self.contact_position_m)
            if self.actuators_at_position(actuator_positions, self.contact_position_m):
                self.state = ExcavationState.LOWER_TO_MAX

        # 3. Lower bucket to maximum excavation depth
        elif self.state == ExcavationState.LOWER_TO_MAX:
            self.spin_bucket()
            self.command_lift(self.max_excavation_position_m)
            if self.actuators_at_position(actuator_positions, self.max_excavation_position_m):
                if odom_position[0] is not None and odom_position[1] is not None:
                    self.start_x = odom_position[0]
                    self.start_y = odom_position[1]
                self.state = ExcavationState.DRIVE_FORWARD

        # 4. Drive forward while drum is spinning
        elif self.state == ExcavationState.DRIVE_FORWARD:
            self.spin_bucket()
            self.drive_forward()
            if self.has_travelled_distance(odom_position, self.drive_distance_m):
                self.state = ExcavationState.STOP_WHEELS_AFTER_DRIVE

        # 5. Stop wheels
        elif self.state == ExcavationState.STOP_WHEELS_AFTER_DRIVE:
            self.stop_wheels()
            self.state = ExcavationState.LIFT_BUCKET

        # 6. Lift bucket while drum is spinning
        elif self.state == ExcavationState.LIFT_BUCKET:
            self.spin_bucket()
            self.command_lift(self.raised_position_m)
            if self.actuators_at_position(actuator_positions, self.raised_position_m):
                self.state = ExcavationState.STOP_BUCKET

        # 7. Stop drum
        elif self.state == ExcavationState.STOP_BUCKET:
            self.stop_wheels()
            self.stop_bucket()
            self.state = ExcavationState.COMPLETE
            self.complete = True

    """Outputs"""
    def drive_forward(self):
        command = Twist()
        command.linear.x = self.drive_speed
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
            self.drum_spin_direction * self.drum_spin_speed
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

    """Feedback"""
    def actuators_at_position(self, actuator_positions, target):
        left, right = actuator_positions

        if left is None or right is None:
            return False

        left_error = abs(left - target)
        right_error = abs(right - target)

        return (
            left_error <= self.position_tolerance_m
            and right_error <= self.position_tolerance_m
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

    """Stop / completion"""
    def stop(self, actuator_positions):
        # Stop autonomous motion and hold the current bucket position
        self.stop_wheels()
        self.stop_bucket()

        left, right = actuator_positions
        if left is not None and right is not None:
            hold_position = (left + right) / 2.0
            self.command_lift(hold_position)

        self.complete = False
        self.state = ExcavationState.COMPLETE

    def is_complete(self):
        return self.complete