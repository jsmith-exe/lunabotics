from dataclasses import dataclass, field

from ..constants import ControllerInputs, MessageOptions, NAV_TOPIC, DRUM_LIFT_TOPIC, DRUM_ROTATION_TOPIC, \
    CmdMeta, GUIInputs


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
    metadata: dict = field(default_factory=dict)

Con = ControllerInputs
GUI = GUIInputs

default_control_map = {
    Con.DPAD_UP: Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    Con.DPAD_DOWN: Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, -1, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    Con.DPAD_RIGHT: Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, -1, metadata={CmdMeta.IS_STEER_MOTOR_COMMAND: True}),
    Con.DPAD_LEFT: Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, metadata={CmdMeta.IS_STEER_MOTOR_COMMAND: True}),
    Con.RIGHT_JOYSTICK_X: Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, -1, metadata={CmdMeta.IS_STEER_MOTOR_COMMAND: True}),
    Con.RIGHT_JOYSTICK_Y: Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),

    Con.CIRCLE: Command(DRUM_ROTATION_TOPIC, MessageOptions.TWIST_LINEAR_X, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),
    Con.SQUARE: Command(DRUM_ROTATION_TOPIC, MessageOptions.TWIST_LINEAR_X, -1, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),
    Con.L2_ANALOGUE_STICK: Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, -1, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),
    Con.R2_ANALOGUE_STICK: Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),

    'w': Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    'a': Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, metadata={CmdMeta.IS_STEER_MOTOR_COMMAND: True}),
    's': Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, -1, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    'd': Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, -1, metadata={CmdMeta.IS_STEER_MOTOR_COMMAND: True}),
    'right': Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),
    'left': Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, -1, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),

    GUI.DRUM_HEIGHT_SLIDER: Command(DRUM_LIFT_TOPIC, MessageOptions.FLOAT)
}
