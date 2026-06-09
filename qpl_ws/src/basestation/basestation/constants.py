import dataclasses
from enum import Enum

class ControlMode(Enum):
    """ Determines what type of controls can be mapped.
    Allows for having multiple modes of controlling the robot (e.g., manual control or smarter steering). """
    TEST = 'test'
    STANDARD = 'standard'

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

class ControllerInputs:
    """ Controller inputs for defining control maps. """
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

NAV_TOPIC = '/cmd_vel_teleop'
DRUM_ROTATION_TOPIC = '/drum_cmd'
DRUM_LIFT_TOPIC = 'cmd_drum_lift'
PUBLISHER_UPDATE_RATE = 2
INVERT_BACKWARDS_STEERING = True
