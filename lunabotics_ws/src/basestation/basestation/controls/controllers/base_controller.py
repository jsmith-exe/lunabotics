from collections.abc import Callable

from lunabotics_ws.src.basestation.basestation.controls.constants import Commands, ControllerInputs
from lunabotics_ws.src.basestation.basestation.controls.controllers.base_station_state import BaseStationState

MINIMUM_ANALOGUE_CHANGE = 0.05 # Changes in analogue values must be at least this much to be sent

class BaseController:
    """
    Base class controller for handling shared implementations of button and analogue inputs.
    """
    def __init__(self, publish_function: Callable, state: BaseStationState):
        """
        :param publish_function: a function that takes in one first argument, the command, and an arbitrary number of arguments.
        :param state: the state object to use.
        """
        self.publish_function = publish_function
        self.state = state
        self.previous_analogue_values = dict()

    def handle_button(self, button: str | ControllerInputs, pressed: bool, control_map: dict) -> None:
        """
        Sends the command associated with the button based on the control map; if the button is released,
        a release command will be sent.
        :param button: the button pressed, some key in the control map.
        :param pressed: if the button was pressed (true) or released (false).
        :param control_map: the control map to use.
        """
        command = control_map.get(button)
        if command is None:
            return

        if pressed:
            self.publish_function(command)
        else:
            # On release, send a STOP signal for that command.
            self.publish_function(Commands.STOP_SIGNAL, command)

    def handle_analogue_input(self, input_: str | ControllerInputs, normalised_value: float, control_map: dict) -> None:
        """
        Sends the command associated with the control map with the value of the analogue input (e.g., mouse or joystick).
        :param input_: the triggered input.
        :param normalised_value: a value from -1 to 1 that represents the value from the analogue input.
        :param control_map: the control map to use.
        """
        command = control_map.get(input_)
        prev_value = self.previous_analogue_values.get(input_)
        if (command is None or
            # Ignore insignificant inputs if there was a previous value and there is a non-zero value from the input.
            (prev_value is not None and abs(normalised_value - prev_value) < MINIMUM_ANALOGUE_CHANGE)):
            return

        self.previous_analogue_values[input_] = normalised_value
        self.publish_function(command, normalised_value)
