from dataclasses import dataclass, field

from ..constants import ControllerInputs, ControlMode, MessageOptions, NAV_TOPIC, DRUM_LIFT_TOPIC, DRUM_ROTATION_TOPIC, \
    MOTOR_THROTTLE_BUTTON_FACTOR


@dataclass
class Command:
    """ Contains sufficient data to represent a change to a twist message.
    :param topic_name: the name of the topic to publish to.
    :param message_option: the value to modify; this might be a float, or a twist attribute.
    :param scale: multiplier for the input value; helpful for handling left and right on the same axis.
    """
    topic_name: str
    message_option: MessageOptions
    scale: float = field(default=1)

Con = ControllerInputs

default_control_map = {
    'mode': ControlMode.STANDARD,
    Con.DPAD_UP: Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, MOTOR_THROTTLE_BUTTON_FACTOR),
    Con.DPAD_DOWN: Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, -MOTOR_THROTTLE_BUTTON_FACTOR),
    Con.DPAD_RIGHT: Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, -1),
    Con.DPAD_LEFT: Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, 1),
    # Con.LEFT_JOYSTICK_X: Command(NAV_TOPIC, TwistOptions.ANGULAR_Z, -1),
    # Con.LEFT_JOYSTICK_Y: Command(NAV_TOPIC, TwistOptions.LINEAR_X),
    Con.RIGHT_JOYSTICK_X: Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, -1),
    Con.RIGHT_JOYSTICK_Y: Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X),

    Con.L2_ANALOGUE_STICK: Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, -1),
    Con.R2_ANALOGUE_STICK: Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT),
    Con.LEFT_JOYSTICK_Y: Command(DRUM_LIFT_TOPIC, MessageOptions.FLOAT),

    'w': Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, MOTOR_THROTTLE_BUTTON_FACTOR),
    'a': Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, 1),
    's': Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, -MOTOR_THROTTLE_BUTTON_FACTOR),
    'd': Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, -1),
    'left': Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, -MOTOR_THROTTLE_BUTTON_FACTOR),
    'right': Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, MOTOR_THROTTLE_BUTTON_FACTOR),
    'up': Command(DRUM_LIFT_TOPIC, MessageOptions.FLOAT, 1),
    'down': Command(DRUM_LIFT_TOPIC, MessageOptions.FLOAT, -1),
}
