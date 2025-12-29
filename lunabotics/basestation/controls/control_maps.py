from lunabotics.basestation.controls.constants import ControllerOptions, ControlMode, Commands

Con = ControllerOptions

default_controller_control_map = {
    'mode': ControlMode.STANDARD,

    Con.DPAD_UP: Commands.FORWARD,
    Con.DPAD_DOWN: Commands.REVERSE,
    Con.DPAD_RIGHT: Commands.RIGHT,
    Con.DPAD_LEFT: Commands.LEFT,
}

default_desktop_control_map = {
    'mode': ControlMode.STANDARD,
    'w': Commands.FORWARD,
    'a': Commands.LEFT,
    's': Commands.REVERSE,
    'd': Commands.RIGHT,
}
