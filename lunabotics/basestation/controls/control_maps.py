from lunabotics.basestation.controls.constants import ControllerInputs, ControlMode, Commands

Con = ControllerInputs

default_controller_control_map = {
    'mode': ControlMode.STANDARD,

    Con.DPAD_UP: Commands.FORWARD,
    Con.DPAD_DOWN: Commands.REVERSE,
    Con.DPAD_RIGHT: Commands.RIGHT,
    Con.DPAD_LEFT: Commands.LEFT,

    Con.LEFT_JOYSTICK_X: Commands.RIGHT,
    Con.LEFT_JOYSTICK_Y: Commands.FORWARD,

    Con.L2_ANALOGUE_STICK: Commands.FORWARD,
}

default_desktop_control_map = {
    'mode': ControlMode.STANDARD,
    'w': Commands.FORWARD,
    'a': Commands.LEFT,
    's': Commands.REVERSE,
    'd': Commands.RIGHT,
}
