"""
Handles setup of a physical controller.
Currently only supports dualsense, may not work on macOS.
Helpful documentation:
- pydualsense docs: https://flok.github.io/pydualsense/ds_main.html
"""

from pydualsense import pydualsense
from lunabotics.basestation.controls.control_maps import default_controller_control_map, Con

control_map = default_controller_control_map # Todo replace with state

def make_on_button_pressed(button):
    if button not in control_map:
        print(f'Invalid button: {button}')
        return lambda: None

    def on_button_pressed(state):
        if not state: return
        command = control_map[button]
        print(command)

    return on_button_pressed

class PhysicalControllerHandler:
    def __init__(self):
        self.stopped = True
        self.controller = None
        controller = pydualsense()
        try:
            controller.init()
        except Exception as err:
            self.onFailToConnect(err)
            return
        self.stopped = False

        controller.circle_pressed += make_on_button_pressed(Con.CIRCLE)
        controller.square_pressed += make_on_button_pressed(Con.SQUARE)
        controller.triangle_pressed += make_on_button_pressed(Con.TRIANGLE)
        controller.cross_pressed += make_on_button_pressed(Con.CROSS)
        controller.option_pressed += make_on_button_pressed(Con.OPTION)
        controller.share_pressed += make_on_button_pressed(Con.SHARE)

        self.controller = controller

    def onFailToConnect(self, err):
        print(f'Exception raised when attempting to connect to controller: {err}')

    def stop(self):
        if self.stopped: return
        self.stopped = True
        self.controller.close()
        print('Controller closed')

if __name__ == '__main__':
    controllerHandler = PhysicalControllerHandler()
    input()
    controllerHandler.stop()
