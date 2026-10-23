from enum import Enum, auto

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class NavDepositionState(Enum):
    STOP_WHEELS = auto()
    RAISE_BUCKET = auto()
    DUMP_REGOLITH = auto()
    STOP_BUCKET = auto()
    COMPLETE = auto()


class NavDepositionSequence:
    """Feedback-based FSM for the deposition sequence used by nav autonomy.

    Nav2 has already put the rover at the berm, so it stands still with the bucket raised
    and spins the drum to dump.
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

        self.raised_position_m = config["raised_position_m"]
        self.dump_duration_s = config["dump_duration_s"]
        self.drum_spin_speed = config["drum_spin_speed"]
        self.drum_spin_direction = config["drum_spin_direction"]
        self.position_tolerance_m = config["position_tolerance_m"]

        self.state = NavDepositionState.COMPLETE
        self.complete = False
        self.dump_start_s = None

    """Sequence lifecycle"""
    def start(self):
        # Start a new deposition sequence
        self.complete = False
        self.state = NavDepositionState.STOP_WHEELS
        self.dump_start_s = None

    def update(self, actuator_positions, now_s):
        # Run one non-blocking update of the FSM
        if self.complete:
            return

        # 1. Stop wheels
        if self.state == NavDepositionState.STOP_WHEELS:
            self.stop_wheels()
            self.state = NavDepositionState.RAISE_BUCKET

        # 2. Make sure the bucket is raised before dumping
        elif self.state == NavDepositionState.RAISE_BUCKET:
            self.stop_wheels()
            self.command_lift(self.raised_position_m)
            if self.actuators_at_position(actuator_positions, self.raised_position_m):
                self.dump_start_s = now_s
                self.state = NavDepositionState.DUMP_REGOLITH

        # 3. Spin drum to dump. Nothing reports when the drum is empty, so this
        # one step is timed.
        elif self.state == NavDepositionState.DUMP_REGOLITH:
            self.stop_wheels()
            self.spin_bucket()
            if now_s - self.dump_start_s >= self.dump_duration_s:
                self.state = NavDepositionState.STOP_BUCKET

        # 4. Stop drum
        elif self.state == NavDepositionState.STOP_BUCKET:
            self.stop_wheels()
            self.stop_bucket()
            self.state = NavDepositionState.COMPLETE
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
        self.state = NavDepositionState.COMPLETE

    def is_complete(self):
        return self.complete
