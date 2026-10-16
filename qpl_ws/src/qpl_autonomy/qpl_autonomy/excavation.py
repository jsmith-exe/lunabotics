from enum import Enum, auto

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class ExcavationState(Enum):
    STOP_WHEELS = auto()
    START_SPIN = auto()
    LOWER_BUCKET = auto()
    RAISE_BUCKET = auto()
    STOP_BUCKET = auto()
    COMPLETE = auto()


class ExcavationSequence:
    """Feedback-based FSM for the excavation sequence.

    Stands still: spins the drum forward, lowers the bucket into the regolith, raises it
    again, then stops the drum.
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

        self.drum_spin_speed = config["drum_spin_speed"]
        self.drum_spin_direction = config["drum_spin_direction"]
        self.lowered_position_m = config["lowered_position_m"]
        self.raised_position_m = config["raised_position_m"]
        self.position_tolerance_m = config["position_tolerance_m"]

        self.state = ExcavationState.COMPLETE
        self.complete = False

    """Sequence lifecycle"""

    def start(self):
        # Start a new excavation sequence
        self.complete = False
        self.state = ExcavationState.STOP_WHEELS

    def update(self, actuator_positions):
        # Run one non-blocking update of the FSM
        if self.complete:
            return

        # 1. Stop wheels
        if self.state == ExcavationState.STOP_WHEELS:
            self.stop_wheels()
            self.state = ExcavationState.START_SPIN

        # 2. Start the drum before it touches the regolith
        elif self.state == ExcavationState.START_SPIN:
            self.stop_wheels()
            self.spin_bucket()
            self.state = ExcavationState.LOWER_BUCKET

        # 3. Lower bucket into the regolith while spinning
        elif self.state == ExcavationState.LOWER_BUCKET:
            self.stop_wheels()
            self.spin_bucket()
            self.command_lift(self.lowered_position_m)
            if self.actuators_at_position(actuator_positions, self.lowered_position_m):
                self.state = ExcavationState.RAISE_BUCKET

        # 4. Raise bucket while still spinning, so it keeps what it collected
        elif self.state == ExcavationState.RAISE_BUCKET:
            self.stop_wheels()
            self.spin_bucket()
            self.command_lift(self.raised_position_m)
            if self.actuators_at_position(actuator_positions, self.raised_position_m):
                self.state = ExcavationState.STOP_BUCKET

        # 5. Stop drum
        elif self.state == ExcavationState.STOP_BUCKET:
            self.stop_wheels()
            self.stop_bucket()
            self.state = ExcavationState.COMPLETE
            self.complete = True

    """Outputs"""
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
