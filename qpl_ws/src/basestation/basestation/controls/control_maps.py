from dataclasses import dataclass, field

from ..constants import ControllerInputs, ControlMode, TwistOptions, NAV_TOPIC, DRUM_LIFT_TOPIC, DRUM_ROTATION_TOPIC

Con = ControllerInputs

@dataclass
class Command:
    """ Contains sufficient data to represent a change to a twist message.
    :param topic_name: the name of the topic to publish to.
    :param twist_option: the twist option to modify.
    :param scale: multiplier for the input value; helpful for handling left and right on the same axis.
    """
    topic_name: str
    twist_option: TwistOptions
    scale: float = field(default=1)

default_control_map = {
    'mode': ControlMode.STANDARD,
    Con.DPAD_UP: Command(NAV_TOPIC, TwistOptions.LINEAR_X),
    Con.DPAD_DOWN: Command(NAV_TOPIC, TwistOptions.LINEAR_X, -1),
    Con.DPAD_RIGHT: Command(NAV_TOPIC, TwistOptions.ANGULAR_Z, -1),
    Con.DPAD_LEFT: Command(NAV_TOPIC, TwistOptions.ANGULAR_Z),
    # Con.LEFT_JOYSTICK_X: Command(NAV_TOPIC, TwistOptions.ANGULAR_Z, -1),
    # Con.LEFT_JOYSTICK_Y: Command(NAV_TOPIC, TwistOptions.LINEAR_X),
    Con.RIGHT_JOYSTICK_X: Command(NAV_TOPIC, TwistOptions.ANGULAR_Z, -1),
    Con.RIGHT_JOYSTICK_Y: Command(NAV_TOPIC, TwistOptions.LINEAR_X),

    Con.L2_ANALOGUE_STICK: Command(DRUM_ROTATION_TOPIC, TwistOptions.ANGULAR_Y, -1),
    Con.R2_ANALOGUE_STICK: Command(DRUM_ROTATION_TOPIC, TwistOptions.ANGULAR_Y),
    Con.LEFT_JOYSTICK_Y: Command(DRUM_LIFT_TOPIC, TwistOptions.LINEAR_Z),

    'w': Command(NAV_TOPIC, TwistOptions.LINEAR_X),
    'a': Command(NAV_TOPIC, TwistOptions.ANGULAR_Z),
    's': Command(NAV_TOPIC, TwistOptions.LINEAR_X, -1),
    'd': Command(NAV_TOPIC, TwistOptions.ANGULAR_Z, -1),
    'left': Command(DRUM_ROTATION_TOPIC, TwistOptions.ANGULAR_Y, -1),
    'right': Command(DRUM_ROTATION_TOPIC, TwistOptions.ANGULAR_Y),
    'up': Command(DRUM_LIFT_TOPIC, TwistOptions.LINEAR_Z),
    'down': Command(DRUM_LIFT_TOPIC, TwistOptions.LINEAR_Z, -1),
}
