from enum import Enum

class ControlMode(Enum):
    """ Determines what type of controls can be mapped.
    Allows for having multiple modes of controlling the robot (e.g., manual control or smarter steering). """
    TEST = 'test'

class Commands(Enum):
    """ Commands that can be sent to the robot. """
    ROUND = 'round'
    FOUR_CORNERS = 'four_corners'
    THREE_CORNERS = 'three_corners'
    X = 'x'
    CHANGE_MODE = 'change_mode'
    BROADCAST = 'broadcast'

class ControllerOptions:
    """ Overridable options for the controller that allow rebinding to different commands. """
    CIRCLE = 'circle'
    SQUARE = 'square'
    TRIANGLE = 'triangle'
    CROSS = 'cross'
    OPTION = 'option'
    SHARE = 'share'
Con = ControllerOptions

default_controller_control_map = {
    'mode': ControlMode.TEST,

    Con.CIRCLE: Commands.ROUND,
    Con.SQUARE: Commands.FOUR_CORNERS,
    Con.TRIANGLE: Commands.THREE_CORNERS,
    Con.CROSS: Commands.X,
    Con.OPTION: Commands.CHANGE_MODE,
    Con.SHARE: Commands.BROADCAST,
}