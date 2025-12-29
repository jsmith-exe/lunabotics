from collections.abc import Callable

from lunabotics.basestation.controls.constants import Commands, ControllerInputs
from lunabotics.basestation.controls.controllers.base_station_state import BaseStationState


class BaseController:
    """
    Base controller for handling shared implementations of button and analogue inputs.
    """
    def __init__(self, publish_function: Callable, state: BaseStationState):
        """
        :param publish_function: a function that takes in one first argument, the command, and an arbitrary number of arguments.
        :param state: the state object to use.
        """
        self.publish_function = publish_function
        self.state = state

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
