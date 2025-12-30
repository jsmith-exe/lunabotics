from lunabotics.basestation.controls.constants import ControllerInputs, ControlMode, Commands

Con = ControllerInputs

default_controller_control_map = {
    'mode': ControlMode.STANDARD,

    Con.TRIANGLE: Commands.TEST,
    Con.CROSS: Commands.TEST,
    Con.SQUARE: Commands.TEST,
    Con.CIRCLE: Commands.TEST,
    Con.OPTION: Commands.TEST,
    Con.SHARE: Commands.TEST,

    Con.DPAD_UP: Commands.FORWARD,
    Con.DPAD_DOWN: Commands.REVERSE,
    Con.DPAD_RIGHT: Commands.RIGHT,
    Con.DPAD_LEFT: Commands.LEFT,

    Con.LEFT_JOYSTICK_BUTTON: Commands.TEST,
    Con.LEFT_JOYSTICK_X: Commands.RIGHT,
    Con.LEFT_JOYSTICK_Y: Commands.FORWARD,
    Con.RIGHT_JOYSTICK_BUTTON: Commands.TEST,
    Con.RIGHT_JOYSTICK_X: Commands.RIGHT,
    Con.RIGHT_JOYSTICK_Y: Commands.FORWARD,

    Con.L1_BUTTON: Commands.TEST,
    Con.R1_BUTTON: Commands.TEST,
    Con.L2_ANALOGUE_STICK: Commands.LEFT,
    Con.R2_ANALOGUE_STICK: Commands.RIGHT,

    Con.PS_BUTTON: Commands.TEST,
    Con.MIC_BUTTON: Commands.TEST,
    Con.TOUCHPAD_BUTTON: Commands.TEST,
}

default_desktop_control_map = {
    'mode': ControlMode.STANDARD,
    'w': Commands.FORWARD,
    'a': Commands.LEFT,
    's': Commands.REVERSE,
    'd': Commands.RIGHT,
}
