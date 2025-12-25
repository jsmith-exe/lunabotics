from lunabotics.basestation.controls.constants import ControllerOptions, ControlMode, Commands

Con = ControllerOptions

default_controller_control_map = {
    'mode': ControlMode.TEST,

    # Con.CIRCLE: Commands.ROUND,
    # Con.SQUARE: Commands.FOUR_CORNERS,
    # Con.TRIANGLE: Commands.THREE_CORNERS,
    # Con.CROSS: Commands.X,
    # Con.OPTION: Commands.CHANGE_MODE,
    # Con.SHARE: Commands.BROADCAST,
}

default_desktop_control_map = {
    'mode': ControlMode.STANDARD,
    'w': Commands.FORWARD,
    'a': Commands.LEFT,
    's': Commands.REVERSE,
    'd': Commands.RIGHT,
}
