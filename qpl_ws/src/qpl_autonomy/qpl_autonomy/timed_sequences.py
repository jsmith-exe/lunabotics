from qpl_autonomy.excavation import ExcavationSequence, ExcavationState
from qpl_autonomy.deposition import DepositionSequence, DepositionState


class TimedExcavationSequence(ExcavationSequence):
    """Excavation FSM where the drive stage runs for a fixed time.

    Expects odom_position to be (time_s, 0.0), so the start "position" the base
    class records is the time the drive stage began.
    """

    def __init__(self, cmd_vel_pub, drum_lift_pub, drum_spin_pub, config):
        super().__init__(cmd_vel_pub, drum_lift_pub, drum_spin_pub, config)
        self.drive_durations_s = {
            ExcavationState.DRIVE_FORWARD: config["drive_duration_s"],
        }

    def has_travelled_distance(self, odom_position, target_distance):
        # target_distance is ignored, the duration for the current state is used instead
        now_s = odom_position[0]
        if now_s is None or self.start_x is None:
            return False

        return now_s - self.start_x >= self.drive_durations_s[self.state]


class TimedDepositionSequence(DepositionSequence):
    """Deposition FSM where the drive stages run for a fixed time.

    Expects odom_position to be (time_s, 0.0), so the start "position" the base
    class records is the time each drive stage began.
    """

    def __init__(self, cmd_vel_pub, drum_lift_pub, drum_spin_pub, config):
        super().__init__(cmd_vel_pub, drum_lift_pub, drum_spin_pub, config)
        self.drive_durations_s = {
            DepositionState.DRIVE_TO_CONSTRUCTION_ZONE: config["drive_to_zone_duration_s"],
            DepositionState.DEPOSIT_REGOLITH: config["deposit_drive_duration_s"],
        }

    def has_travelled_distance(self, odom_position, target_distance):
        # target_distance is ignored, the duration for the current state is used instead
        now_s = odom_position[0]
        if now_s is None or self.start_x is None:
            return False

        return now_s - self.start_x >= self.drive_durations_s[self.state]
