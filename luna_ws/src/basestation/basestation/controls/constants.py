from enum import Enum

class ControlMode(Enum):
    """ Determines what type of controls can be mapped.
    Allows for having multiple modes of controlling the robot (e.g., manual control or smarter steering). """
    TEST = 'test'
    STANDARD = 'standard'

class Commands(Enum):
    """ Commands that can be sent to the robot. """
    FORWARD = 'MVF'
    RIGHT = 'MVR'
    LEFT = 'MVL'
    REVERSE = 'MVB'
    STOP_SIGNAL = 'STP'
    TEST = 'TST'

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
