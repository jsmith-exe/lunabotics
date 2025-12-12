from enum import Enum

from pydualsense import pydualsense

class ControlTypes(Enum):
    TEST = 'test'

class TestCommands(Enum):
    CIRCLE = 'circle'
    SQUARE = 'square'
    TRIANGLE = 'triangle'
    CROSS = 'cross'
    OPTION = 'option'
    SHARE = 'share'

default_controller_control_map = {
    'mode': ControlTypes.TEST,
    'circle': TestCommands.CIRCLE,
    'square': TestCommands.SQUARE,
    'triangle': TestCommands.TRIANGLE,
    'cross': TestCommands.CROSS,
    'option': TestCommands.OPTION,
    'share': TestCommands.SHARE
}

def make_on_button_pressed(button):
    if button not in default_controller_control_map:
        print(f'Invalid button: {button}')
        return lambda: None

    def on_button_pressed(state):
        if not state: return
        command = default_controller_control_map[button]
        print(command)

    return on_button_pressed

# https://flok.github.io/pydualsense/ds_main.html
class PhysicalControllerHandler:
    def __init__(self):
        self.closed = False
        controller = pydualsense()
        controller.init()

        controller.circle_pressed += make_on_button_pressed('circle')
        controller.square_pressed += make_on_button_pressed('square')
        controller.triangle_pressed += make_on_button_pressed('triangle')
        controller.cross_pressed += make_on_button_pressed('cross')
        controller.option_pressed += make_on_button_pressed('option')
        controller.share_pressed += make_on_button_pressed('share')

        self.controller = controller

    def close(self):
        self.closed = True
        self.controller.close()
        print('Controller closed')

    def __del__(self):
        if self.closed: return
        self.close()

if __name__ == '__main__':
    controllerHandler = PhysicalControllerHandler()
    input()
    controllerHandler.close()