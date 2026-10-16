from enum import Enum, auto
from math import hypot

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class DepositionState(Enum):
    DRIVE_TO_CONSTRUCTION_ZONE = auto()
    STOP_WHEELS = auto()
    RAISE_BUCKET = auto()
    DUMP_REGOLITH = auto()
    STOP_BUCKET = auto()
    COMPLETE = auto()


class DepositionSequence:
    """Feedback-based FSM for the competition deposition sequence.

    Stands still with the bucket raised and spins the drum backwards to dump.
    """

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
        self.raised_position_m = config["raised_position_m"]
        self.dump_duration_s = config["dump_duration_s"]
        self.drum_spin_speed = config["drum_spin_speed"]
        self.drum_spin_direction = config["drum_spin_direction"]
        self.position_tolerance_m = config["position_tolerance_m"]

        self.state = DepositionState.COMPLETE
        self.complete = False

        self.start_x = None
        self.start_y = None
        self.dump_start_s = None

    """Sequence lifecycle"""
    def start(self, odom_position, drive_to_zone=True):
        # Start a new deposition sequence. drive_to_zone=False skips the blind drive in,
        # for when navigation has already put the rover at the berm.
        self.complete = False
        self.state = (
            DepositionState.DRIVE_TO_CONSTRUCTION_ZONE if drive_to_zone
            else DepositionState.STOP_WHEELS
        )
        self.start_x = None
        self.start_y = None
        self.dump_start_s = None
        if odom_position[0] is not None and odom_position[1] is not None:
            self.start_x = odom_position[0]
            self.start_y = odom_position[1]

    def update(self, actuator_positions, odom_position, now_s):
        # Run one non-blocking update of the FSM
        if self.complete:
            return

        # 1. Drive into construction zone (blind DEPOSIT only)
        if self.state == DepositionState.DRIVE_TO_CONSTRUCTION_ZONE:
            self.drive_forward(self.drive_to_zone_speed)
            if self.has_travelled_distance(odom_position, self.drive_to_zone_distance_m):
                self.state = DepositionState.STOP_WHEELS

        # 2. Stop wheels
        elif self.state == DepositionState.STOP_WHEELS:
            self.stop_wheels()
            self.state = DepositionState.RAISE_BUCKET

        # 3. Make sure the bucket is raised before dumping
        elif self.state == DepositionState.RAISE_BUCKET:
            self.stop_wheels()
            self.command_lift(self.raised_position_m)
            if self.actuators_at_position(actuator_positions, self.raised_position_m):
                self.dump_start_s = now_s
                self.state = DepositionState.DUMP_REGOLITH

        # 4. Spin drum backwards to dump. Nothing reports when the drum is empty, so this
        # one step is timed.
        elif self.state == DepositionState.DUMP_REGOLITH:
            self.stop_wheels()
            self.spin_bucket()
            if now_s - self.dump_start_s >= self.dump_duration_s:
                self.state = DepositionState.STOP_BUCKET

        # 5. Stop drum
        elif self.state == DepositionState.STOP_BUCKET:
            self.stop_wheels()
            self.stop_bucket()
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
