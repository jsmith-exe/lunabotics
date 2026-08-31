from collections.abc import Callable

from ...constants import ControllerInputs, CmdMeta, GUIInputs
from .base_station_state import BaseStationState
from ..control_maps import Command

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

    def handle_button(self, button: str | ControllerInputs | GUIInputs, pressed: bool) -> None:
        """
        Sends the command associated with the button based on the control map; if the button is released,
        a release command will be sent.
        :param button: the button pressed, some key in the control map.
        :param pressed: if the button was pressed (true) or released (false).
        :param control_map: the control map to use.
        """
        if not self.state.teleop_enabled: return
        command: Command = self.state.control_map.get(button)
        if command is None:
            return

        if pressed:
            self.publish_function(command.topic_name, command.message_option, self._post_process_value(1, command))
        else:
            # On release, send a STOP signal for that command.
            self.publish_function(command.topic_name, command.message_option, 0)

    def handle_analogue_input(self, input_: str | ControllerInputs | GUIInputs, value: float) -> None:
        """
        Sends the command associated with the control map with the value of the analogue input (e.g., mouse or joystick).
        :param input_: the triggered input.
        :param value: a value from the analogue input.
        :param control_map: the control map to use.
        """
        if not self.state.teleop_enabled: return
        command: Command = self.state.control_map.get(input_)
        prev_value = self.previous_analogue_values.get(input_)
        if (command is None or
            # Ignore insignificant inputs if there was a previous value and there is a non-zero value from the input.
            (prev_value is not None and abs(value - prev_value) < MINIMUM_ANALOGUE_CHANGE)):
            return

        normalised_value = self._post_process_value(value, command)
        self.previous_analogue_values[input_] = normalised_value
        self.publish_function(command.topic_name, command.message_option, normalised_value)

    def _post_process_value(self, value: float, command: Command):
        # Apply motor factors depending on command metadata
        if command.metadata.get(CmdMeta.IS_DRIVE_MOTOR_COMMAND):
            value *= self.state.motor_drive_button_factor
        elif command.metadata.get(CmdMeta.IS_STEER_MOTOR_COMMAND):
            value *= self.state.motor_steer_button_factor
        elif command.metadata.get(CmdMeta.IS_DRUM_MOTOR_COMMAND):
            value *= self.state.motor_drum_button_factor

        return value * command.scale
