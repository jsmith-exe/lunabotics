from dataclasses import dataclass, field

from .constants import ControllerInputs, ControlMode, TwistOptions, NAV_TOPIC

Con = ControllerInputs

@dataclass
class Command:
    """ TODO DOCUMENT """
    topic_name: str
    twist_option: TwistOptions
    scale: float = field(default=1) # Multiplier for the input value; helpful for handling left and right on the same axis.

default_control_map = {
    'mode': ControlMode.STANDARD,
    Con.DPAD_UP: Command(NAV_TOPIC, TwistOptions.LINEAR_X),
    Con.DPAD_DOWN: Command(NAV_TOPIC, TwistOptions.LINEAR_X, -1),
    Con.DPAD_RIGHT: Command(NAV_TOPIC, TwistOptions.ANGULAR_X),
    Con.DPAD_LEFT: Command(NAV_TOPIC, TwistOptions.ANGULAR_X, -1),

    Con.LEFT_JOYSTICK_X: Command(NAV_TOPIC, TwistOptions.ANGULAR_X),
    Con.LEFT_JOYSTICK_Y: Command(NAV_TOPIC, TwistOptions.LINEAR_X),
    Con.RIGHT_JOYSTICK_X: Command(NAV_TOPIC, TwistOptions.ANGULAR_X),
    Con.RIGHT_JOYSTICK_Y: Command(NAV_TOPIC, TwistOptions.LINEAR_X),

    Con.L2_ANALOGUE_STICK: Command(NAV_TOPIC, TwistOptions.ANGULAR_X, -1),
    Con.R2_ANALOGUE_STICK: Command(NAV_TOPIC, TwistOptions.ANGULAR_X),

    'w': Command(NAV_TOPIC, TwistOptions.LINEAR_X),
    'a': Command(NAV_TOPIC, TwistOptions.ANGULAR_X, -1),
    's': Command(NAV_TOPIC, TwistOptions.LINEAR_X, -1),
    'd': Command(NAV_TOPIC, TwistOptions.ANGULAR_X),
}
