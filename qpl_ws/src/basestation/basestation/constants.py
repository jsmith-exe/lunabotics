from enum import Enum
from pathlib import Path

NAV_TOPIC = '/cmd_vel_teleop'
DRUM_ROTATION_TOPIC = '/drum_spin_control/teleop'
DRUM_LIFT_TOPIC = '/drum_lift_control/teleop'
AUTONOMY_COMMAND_TOPIC = '/autonomy/command'
NAV_AUTONOMY_COMMAND_TOPIC = '/nav_autonomy/command'
NAV_AUTONOMY_STATE_TOPIC = '/nav_autonomy/state'
JOINT_STATES_TOPIC = '/joint_states'
DRUM_LIFT_JOINTS = ('left_linear_actuator_joint', 'right_linear_actuator_joint')
PUBLISHER_UPDATE_RATE = 5
INVERT_BACKWARDS_STEERING = False
DEFAULT_MOTOR_DRIVE_BUTTON_FACTOR = 0.3
DEFAULT_MOTOR_STEER_BUTTON_FACTOR = 0.25
DEFAULT_MOTOR_DRUM_BUTTON_FACTOR = 0.4
# Must match diff_cont's wheel_separation (qpl_rover/config/my_controllers.yaml)
TANK_WHEEL_SEPARATION = 1.161
CANBUS_CONFIG_PATH = Path(__file__).resolve().parents[2] / 'diffdrive_canbus' / 'hardware' / 'can_config.yaml'
ICON_PATH = Path(__file__).resolve().parents[0] / 'ui' / 'icon.png'

class MessageOptions(Enum):
    """ Commands that can be sent to the robot. """
    FLOAT = 'float'
    TWIST_LINEAR_X = 'twist_linear_x'
    TWIST_LINEAR_Y = 'twist_linear_y'
    TWIST_LINEAR_Z = 'twist_linear_z'
    TWIST_ANGULAR_X = 'twist_angular_x'
    TWIST_ANGULAR_Y = 'twist_angular_y'
    TWIST_ANGULAR_Z = 'twist_angular_z'
    TANK_LEFT = 'tank_left'
    TANK_RIGHT = 'tank_right'

class CmdMeta(Enum):
    """
    Command metadata allows for separation of concerns, allowing all description to belong in the control mapping.
    """
    IS_DRIVE_MOTOR_COMMAND = 'is_drive_motor_command'
    IS_STEER_MOTOR_COMMAND = 'is_steer_motor_command'
    IS_DRUM_MOTOR_COMMAND = 'is_drum_motor_command'
    # While the button is held, the topic's current value moves by `scale` per second; releasing leaves it in place.
    # Float topics only.
    IS_INCREMENT_COMMAND = 'is_increment_command'
    # Bounds that increment commands are clamped to.
    MIN_VALUE = 'min_value'
    MAX_VALUE = 'max_value'

class ControllerInputs:
    """ Controller inputs for defining control maps. This is intentionally not an enum to make accessing values easier. """
    CIRCLE = 'circle'
    SQUARE = 'square'
    TRIANGLE = 'triangle'
    CROSS = 'cross'
    OPTION = 'option'
    SHARE = 'share'

    DPAD_UP = 'dpad_up'
    DPAD_DOWN = 'dpad_down'
    DPAD_RIGHT = 'dpad_right'
    DPAD_LEFT = 'dpad_left'

    LEFT_JOYSTICK_BUTTON = 'left_joystick_btn'
    LEFT_JOYSTICK_X = 'left_joystick_x'
    LEFT_JOYSTICK_Y = 'left_joystick_y'
    RIGHT_JOYSTICK_BUTTON = 'right_joystick_btn'
    RIGHT_JOYSTICK_X = 'right_joystick_x'
    RIGHT_JOYSTICK_Y = 'right_joystick_y'

    L1_BUTTON = 'l1'
    R1_BUTTON = 'r1'
    L2_ANALOGUE_STICK = 'l2'
    R2_ANALOGUE_STICK = 'r2'

    PS_BUTTON = 'ps_button'
    MIC_BUTTON = 'microphone_button'
    TOUCHPAD_BUTTON = 'touchpad_button'

class GUIInputs:
    DRUM_HEIGHT_SLIDER = 'drum_height_slider'
