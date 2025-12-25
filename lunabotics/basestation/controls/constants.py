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

class ControllerOptions:
    """ Overridable options for the controller that allow rebinding to different commands. """
    CIRCLE = 'circle'
    SQUARE = 'square'
    TRIANGLE = 'triangle'
    CROSS = 'cross'
    OPTION = 'option'
    SHARE = 'share'