from enum import Enum

class MessageOptions(Enum):
    """ Commands that can be sent to the robot. """
    FLOAT = 'float'
    TWIST_LINEAR_X = 'twist_linear_x'
    TWIST_LINEAR_Y = 'twist_linear_y'
    TWIST_LINEAR_Z = 'twist_linear_z'
    TWIST_ANGULAR_X = 'twist_angular_x'
    TWIST_ANGULAR_Y = 'twist_angular_y'
    TWIST_ANGULAR_Z = 'twist_angular_z'

    def get_topic_type(self):
        """ Gets the type of the message option, either 'twist' or 'float'. """
        return self.value[:5]

class CmdMeta(Enum):
    """
    Command metadata allows for separation of concerns, allowing all description to belong in the control mapping.
    """
    IS_DRIVE_MOTOR_COMMAND = 'is_drive_motor_command'
    IS_STEER_MOTOR_COMMAND = 'is_steer_motor_command'
    IS_DRUM_MOTOR_COMMAND = 'is_drum_motor_command'

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

NAV_TOPIC = '/cmd_vel_teleop'
DRUM_ROTATION_TOPIC = '/drum_spin_control/teleop'
DRUM_LIFT_TOPIC = '/drum_lift_control/teleop'
PUBLISHER_UPDATE_RATE = 2
INVERT_BACKWARDS_STEERING = True
DEFAULT_MOTOR_DRIVE_BUTTON_FACTOR = 0.3
DEFAULT_MOTOR_STEER_BUTTON_FACTOR = 0.25
DEFAULT_MOTOR_DRUM_BUTTON_FACTOR = 0.4
