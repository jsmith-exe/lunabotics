from dataclasses import dataclass, field

import yaml

from .constants import ControllerInputs, MessageOptions, NAV_TOPIC, DRUM_LIFT_TOPIC, DRUM_ROTATION_TOPIC, \
    CmdMeta, GUIInputs, CANBUS_CONFIG_PATH

DRUM_LIFT_SPEED = 0.05 # m/s, while a drum lift button is held

with open(CANBUS_CONFIG_PATH, "r") as file:
    _drum_config = yaml.safe_load(file)['drum']
DRUM_LIFT_LIMITS = {CmdMeta.MIN_VALUE: _drum_config['min_lift_mm'] / 1000,
                    CmdMeta.MAX_VALUE: _drum_config['max_lift_mm'] / 1000}
DRUM_LIFT_METADATA = {CmdMeta.IS_INCREMENT_COMMAND: True} | DRUM_LIFT_LIMITS


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
    Con.DPAD_UP: Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, -1, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    Con.DPAD_DOWN: Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, 1, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    Con.DPAD_RIGHT: Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, 1, metadata={CmdMeta.IS_STEER_MOTOR_COMMAND: True}),
    Con.DPAD_LEFT: Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, -1, metadata={CmdMeta.IS_STEER_MOTOR_COMMAND: True}),
    # Tank drive via joysticksq
    Con.LEFT_JOYSTICK_Y: Command(NAV_TOPIC, MessageOptions.TANK_LEFT, -1, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    Con.RIGHT_JOYSTICK_Y: Command(NAV_TOPIC, MessageOptions.TANK_RIGHT, -1, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),

    Con.CIRCLE: Command(DRUM_ROTATION_TOPIC, MessageOptions.TWIST_LINEAR_X, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),
    Con.SQUARE: Command(DRUM_ROTATION_TOPIC, MessageOptions.TWIST_LINEAR_X, -1, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),
    Con.L2_ANALOGUE_STICK: Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, -1, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),
    Con.R2_ANALOGUE_STICK: Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),
    Con.TRIANGLE: Command(DRUM_LIFT_TOPIC, MessageOptions.FLOAT, DRUM_LIFT_SPEED, metadata=DRUM_LIFT_METADATA),
    Con.CROSS: Command(DRUM_LIFT_TOPIC, MessageOptions.FLOAT, -DRUM_LIFT_SPEED, metadata=DRUM_LIFT_METADATA),

    's': Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    'a': Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, metadata={CmdMeta.IS_STEER_MOTOR_COMMAND: True}),
    'w': Command(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, -1, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    'd': Command(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, -1, metadata={CmdMeta.IS_STEER_MOTOR_COMMAND: True}),
    'right': Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),
    'left': Command(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, -1, metadata={CmdMeta.IS_DRUM_MOTOR_COMMAND: True}),
    'up': Command(DRUM_LIFT_TOPIC, MessageOptions.FLOAT, DRUM_LIFT_SPEED, metadata=DRUM_LIFT_METADATA),
    'down': Command(DRUM_LIFT_TOPIC, MessageOptions.FLOAT, -DRUM_LIFT_SPEED, metadata=DRUM_LIFT_METADATA),
    # Tank drive
    'v': Command(NAV_TOPIC, MessageOptions.TANK_LEFT, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    'f': Command(NAV_TOPIC, MessageOptions.TANK_LEFT, -1, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    'n': Command(NAV_TOPIC, MessageOptions.TANK_RIGHT, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),
    'j': Command(NAV_TOPIC, MessageOptions.TANK_RIGHT, -1, metadata={CmdMeta.IS_DRIVE_MOTOR_COMMAND: True}),


    GUI.DRUM_HEIGHT_SLIDER: Command(DRUM_LIFT_TOPIC, MessageOptions.FLOAT)
}
