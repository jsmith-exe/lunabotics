"""
Handles setup of a physical controller.
Currently only supports dualsense, may not work on macOS.
Helpful documentation:
- pydualsense docs: https://flok.github.io/pydualsense/ds_main.html
"""
from collections.abc import Callable

from pydualsense import pydualsense

from lunabotics.basestation.controls.control_maps import Con
from lunabotics.basestation.controls.controllers.base_controller import BaseController
from lunabotics.basestation.controls.controllers.base_station_state import BaseStationState

class PhysicalControllerHandler(BaseController):
    """ Translates controller inputs to commands. """
    def __init__(self, publish_function, state: BaseStationState):
        super().__init__(publish_function, state)
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
        """ Assigns event listeners for handling inputs. """
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

    def make_on_button_pressed(self, button: Con | str) -> Callable:
        """
        Creates a method for handling the button passed.
        :param button: the button to handle.
        :return: a function that will be called; must take a boolean parameter.
        """
        def on_button_pressed(pressed):
            self.handle_button(button, pressed, self.state.controller_control_map)
        return on_button_pressed

    def onFailToConnect(self, err: Exception):
        print(f'Exception raised when attempting to connect to controller: {err}')

    def stop(self):
        if self.stopped: return
        self.stopped = True
        self.controller.close()
        print('Controller closed')

if __name__ == '__main__':
    controllerHandler = PhysicalControllerHandler(print, BaseStationState())
    try:
        input()
    except KeyboardInterrupt:
        pass
    controllerHandler.stop()
