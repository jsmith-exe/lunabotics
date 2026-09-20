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

        self.drive_to_zone_speed = config["drive_to_zone_speed"]
        self.drive_to_zone_distance_m = config["drive_to_zone_distance_m"]
        self.deposit_drive_speed = config["deposit_drive_speed"]
        self.deposit_drive_distance_m = config["deposit_drive_distance_m"]
        self.max_height_position_m = config["max_height_position_m"]
        self.normal_height_position_m = config["normal_height_position_m"]
        self.drum_spin_speed = config["drum_spin_speed"]
        self.drum_spin_direction = config["drum_spin_direction"]
        self.position_tolerance_m = config["position_tolerance_m"]

        self.state = DepositionState.COMPLETE
        self.complete = False

        self.start_x = None
        self.start_y = None

    """Sequence lifecycle"""
    def start(self, odom_position):
        # Start a new deposition sequence
        self.complete = False
        self.state = DepositionState.DRIVE_TO_CONSTRUCTION_ZONE
        self.start_x = None
        self.start_y = None
        if odom_position[0] is not None and odom_position[1] is not None:
            self.start_x = odom_position[0]
            self.start_y = odom_position[1]

    def update(self, actuator_positions, odom_position):
        # Run one non-blocking update of the FSM
        if self.complete:
            return

        # 1. Drive into construction zone
        if self.state == DepositionState.DRIVE_TO_CONSTRUCTION_ZONE:
            self.drive_forward(self.drive_to_zone_speed)
            if self.has_travelled_distance(odom_position, self.drive_to_zone_distance_m):
                self.stop_wheels()
                self.start_x = None
                self.start_y = None
                self.state = DepositionState.LIFT_BUCKET

        # 2. Lift bucket to maximum height
        elif self.state == DepositionState.LIFT_BUCKET:
            self.stop_wheels()
            self.command_lift(self.max_height_position_m)
            if self.actuators_at_position(actuator_positions, self.max_height_position_m):
                if odom_position[0] is not None and odom_position[1] is not None:
                    self.start_x = odom_position[0]
                    self.start_y = odom_position[1]
                self.state = DepositionState.DEPOSIT_REGOLITH

        # 3. Spin drum and slowly drive forward
        elif self.state == DepositionState.DEPOSIT_REGOLITH:
            self.spin_bucket()
            self.drive_forward(self.deposit_drive_speed)
            if self.has_travelled_distance(odom_position, self.deposit_drive_distance_m):
                self.state = DepositionState.STOP_WHEELS

        # 4. Stop wheels
        elif self.state == DepositionState.STOP_WHEELS:
            self.stop_wheels()
            self.state = DepositionState.STOP_BUCKET

        # 5. Stop drum
        elif self.state == DepositionState.STOP_BUCKET:
            self.stop_wheels()
            self.stop_bucket()
            self.state = DepositionState.RETURN_TO_NORMAL_HEIGHT

        # 6. Return bucket to normal height
        elif self.state == DepositionState.RETURN_TO_NORMAL_HEIGHT:
            self.stop_wheels()
            self.stop_bucket()
            self.command_lift(self.normal_height_position_m)
            if self.actuators_at_position(actuator_positions, self.normal_height_position_m):
                self.state = DepositionState.COMPLETE
                self.complete = True

    """Outputs"""
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

    """Stop"""
    def stop(self, actuator_positions):
        # Stop autonomous motion and hold the current bucket position
        self.stop_wheels()
        self.stop_bucket()

        left, right = actuator_positions
        if left is not None and right is not None:
            hold_position = (left + right) / 2.0
            self.command_lift(hold_position)

        self.complete = False
        self.state = DepositionState.COMPLETE

    def is_complete(self):
        return self.complete