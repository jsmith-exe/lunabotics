"""
Handles setup of a physical controller.
Currently only supports dualsense, may not work on macOS.
Helpful documentation:
- pydualsense docs: https://flok.github.io/pydualsense/ds_main.html
"""

from pydualsense import pydualsense

from lunabotics.basestation.controls.constants import Commands
from lunabotics.basestation.controls.control_maps import default_controller_control_map, Con

control_map = default_controller_control_map # Todo replace with state

class PhysicalControllerHandler:
    def __init__(self, publish_function):
        self.publish_function = publish_function
        self.stopped = True
        self.controller = None

        controller = pydualsense()
        try:
            controller.init()
        except Exception as err:
            self.onFailToConnect(err)
            return

        self.stopped = False
        self.controller = controller
        self.add_event_listeners()

    def add_event_listeners(self):
        controller = self.controller
        controller.circle_pressed += self.make_on_button_pressed(Con.CIRCLE)
        controller.square_pressed += self.make_on_button_pressed(Con.SQUARE)
        controller.triangle_pressed += self.make_on_button_pressed(Con.TRIANGLE)
        controller.cross_pressed += self.make_on_button_pressed(Con.CROSS)
        controller.option_pressed += self.make_on_button_pressed(Con.OPTION)
        controller.share_pressed += self.make_on_button_pressed(Con.SHARE)

        controller.dpad_up += self.make_on_button_pressed(Con.DPAD_UP)
        controller.dpad_down += self.make_on_button_pressed(Con.DPAD_DOWN)
        controller.dpad_right += self.make_on_button_pressed(Con.DPAD_RIGHT)
        controller.dpad_left += self.make_on_button_pressed(Con.DPAD_LEFT)

    def make_on_button_pressed(self, button):
        if button not in control_map:
            print(f'Invalid button: {button}')
            return lambda *_: None # Take any arguments, return None

        def on_button_pressed(pressed):
            command = control_map.get(button)
            if pressed:
                self.publish_function(command)
            else:
                # On release, send a STOP signal for that command.
                self.publish_function(Commands.STOP_SIGNAL, command)

        return on_button_pressed

    def onFailToConnect(self, err):
        print(f'Exception raised when attempting to connect to controller: {err}')

    def stop(self):
        if self.stopped: return
        self.stopped = True
        self.controller.close()
        print('Controller closed')

if __name__ == '__main__':
    controllerHandler = PhysicalControllerHandler(print)
    input()
    controllerHandler.stop()
