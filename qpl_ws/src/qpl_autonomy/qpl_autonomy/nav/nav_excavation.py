from enum import Enum, auto

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class NavExcavationState(Enum):
    STOP_WHEELS = auto()
    START_SPIN = auto()
    LOWER_BUCKET = auto()
    DIG = auto()
    RAISE_BUCKET = auto()
    STOP_BUCKET = auto()
    COMPLETE = auto()


class NavExcavationSequence:
    """Feedback-based FSM for the excavation sequence used by nav autonomy.

    Stands still: spins the drum forward, lowers the bucket into the regolith, holds it there
    for dig_duration_s, raises it again, then stops the drum.

    Started with driven=True (AUTO_TRENCH), the dig step leaves the wheels to the caller, which
    drives the rover with the bucket down and calls finish_dig() at the end of the trench.
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
        self.dig_duration_s = config["dig_duration_s"]
        self.raised_position_m = config["raised_position_m"]
        self.position_tolerance_m = config["position_tolerance_m"]

        self.state = NavExcavationState.COMPLETE
        self.complete = False
        self.dig_start_s = None
        self.driven = False

    """Sequence lifecycle"""

    def start(self, driven=False):
        # Start a new excavation sequence
        self.complete = False
        self.state = NavExcavationState.STOP_WHEELS
        self.dig_start_s = None
        self.driven = driven

    def finish_dig(self):
        # End a driven dig step: raise the bucket next
        if self.state == NavExcavationState.DIG:
            self.state = NavExcavationState.RAISE_BUCKET

    def update(self, actuator_positions, now_s):
        # Run one non-blocking update of the FSM
        if self.complete:
            return

        # 1. Stop wheels
        if self.state == NavExcavationState.STOP_WHEELS:
            self.stop_wheels()
            self.state = NavExcavationState.START_SPIN

        # 2. Start the drum before it touches the regolith
        elif self.state == NavExcavationState.START_SPIN:
            self.stop_wheels()
            self.spin_bucket()
            self.state = NavExcavationState.LOWER_BUCKET

        # 3. Lower bucket into the regolith while spinning
        elif self.state == NavExcavationState.LOWER_BUCKET:
            self.stop_wheels()
            self.spin_bucket()
            self.command_lift(self.lowered_position_m)
            if self.actuators_at_position(actuator_positions, self.lowered_position_m):
                self.dig_start_s = now_s
                self.state = NavExcavationState.DIG

        # 4. Hold the bucket down and keep spinning. Nothing reports when the bucket is full,
        # so this step is timed (or, driven, ended by the caller).
        elif self.state == NavExcavationState.DIG:
            if not self.driven:
                self.stop_wheels()
            self.spin_bucket()
            self.command_lift(self.lowered_position_m)
            if not self.driven and now_s - self.dig_start_s >= self.dig_duration_s:
                self.state = NavExcavationState.RAISE_BUCKET

        # 5. Raise bucket while still spinning, so it keeps what it collected
        elif self.state == NavExcavationState.RAISE_BUCKET:
            self.stop_wheels()
            self.spin_bucket()
            self.command_lift(self.raised_position_m)
            if self.actuators_at_position(actuator_positions, self.raised_position_m):
                self.state = NavExcavationState.STOP_BUCKET

        # 6. Stop drum
        elif self.state == NavExcavationState.STOP_BUCKET:
            self.stop_wheels()
            self.stop_bucket()
            self.state = NavExcavationState.COMPLETE
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
        self.state = NavExcavationState.COMPLETE

    def is_complete(self):
        return self.complete
