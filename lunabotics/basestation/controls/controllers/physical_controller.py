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

JOYSTICK_DEAD_ZONE = 0.1

ANALOGUE_STICK_RAW_MAX = 255
JOYSTICK_NEGATIVE_MAX = 128
JOYSTICK_POSITIVE_MAX = 127

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
        controller.circle_pressed += self.make_button_listener(Con.CIRCLE)
        controller.square_pressed += self.make_button_listener(Con.SQUARE)
        controller.triangle_pressed += self.make_button_listener(Con.TRIANGLE)
        controller.cross_pressed += self.make_button_listener(Con.CROSS)
        controller.option_pressed += self.make_button_listener(Con.OPTION)
        controller.share_pressed += self.make_button_listener(Con.SHARE)

        controller.dpad_up += self.make_button_listener(Con.DPAD_UP)
        controller.dpad_down += self.make_button_listener(Con.DPAD_DOWN)
        controller.dpad_right += self.make_button_listener(Con.DPAD_RIGHT)
        controller.dpad_left += self.make_button_listener(Con.DPAD_LEFT)

        controller.left_joystick_changed += self.make_joystick_listener(Con.LEFT_JOYSTICK_X, Con.LEFT_JOYSTICK_Y)

        controller.l2_value_changed += self.make_analogue_input_listener(Con.L2_ANALOGUE_STICK)

    def make_button_listener(self, button: Con | str) -> Callable:
        """
        Creates a method for handling the button passed.
        :param button: the button to handle.
        :return: a function that will be called; must take a boolean parameter.
        """
        def on_button_pressed(pressed):
            self.handle_button(button, pressed, self.state.controller_control_map)
        return on_button_pressed

    def make_analogue_input_listener(self, analogue_stick: Con | str) -> Callable:
        """ Similar to make_button_listener """
        def on_input_changed(value):
            self.handle_analogue_input(analogue_stick, value / ANALOGUE_STICK_RAW_MAX, self.state.controller_control_map)
        return on_input_changed

    def make_joystick_listener(self, joystick_x: Con | str, joystick_y: Con | str):
        """ Similar to make_button_listener """
        def on_input_changed(x, y):
            handle_joystick_axis(x, joystick_x)
            handle_joystick_axis(-y, joystick_y) # Invert so that up is positive

        def handle_joystick_axis(joystick_value, joystick):
            joystick_value = normalise(joystick_value)
            if abs(joystick_value) < JOYSTICK_DEAD_ZONE:
                joystick_value = 0
            self.handle_analogue_input(joystick, joystick_value, self.state.controller_control_map)

        def normalise(joystick_value: int):
            """ Converts joystick values from [-128, 127] to [-1, 1] """
            if joystick_value < 0:
                return joystick_value / JOYSTICK_NEGATIVE_MAX
            return joystick_value / JOYSTICK_POSITIVE_MAX

        return on_input_changed

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
