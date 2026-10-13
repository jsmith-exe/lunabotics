from collections.abc import Callable
from math import inf
from threading import Event, Thread

from ..constants import ControllerInputs, CmdMeta, GUIInputs
from ..base_station_state import BaseStationState
from ..control_maps import Command

MINIMUM_ANALOGUE_CHANGE = 0.05 # Changes in analogue values must be at least this much to be sent
INCREMENT_UPDATE_RATE = 20 # Hz; how often a held increment command steps its value

class BaseController:
    """
    Base class controller for handling shared implementations of button and analogue inputs.
    """
    def __init__(self, publish_function: Callable, state: BaseStationState, minimum_analogue_change: float = MINIMUM_ANALOGUE_CHANGE):
        """
        :param publish_function: a function that takes in one first argument, the command, and an arbitrary number of arguments.
        :param state: the state object to use.
        """
        self.publish_function = publish_function
        self.state = state
        self.previous_analogue_values = dict()
        self.minimum_analogue_change = minimum_analogue_change
        # Held increment buttons, mapped to the event that stops their stepping thread.
        self._held_increments: dict[str, Event] = {}

    def handle_button(self, button: str | ControllerInputs | GUIInputs, pressed: bool) -> None:
        """
        Sends the command associated with the button based on the control map; if the button is released,
        a release command will be sent.
        :param button: the button pressed, some key in the control map.
        :param pressed: if the button was pressed (true) or released (false).
        """
        if not self.state.teleop_enabled: return
        command: Command = self.state.control_map.get(button)
        if command is None:
            return

        if command.metadata.get(CmdMeta.IS_INCREMENT_COMMAND):
            self._handle_increment_button(button, command, pressed)
        elif pressed:
            self.publish_function(command.topic_name, command.message_option, self._post_process_value(1, command))
        else:
            # On release, send a STOP signal for that command.
            self.publish_function(command.topic_name, command.message_option, 0)

    def _handle_increment_button(self, button: str, command: Command, pressed: bool) -> None:
        """ Steps the value in a background thread while held; releasing stops stepping, without sending a stop. """
        # Also stops a thread left over from a release that was ignored while teleop was disabled.
        held = self._held_increments.pop(button, None)
        if held is not None:
            held.set()
        if not pressed:
            return

        stop = Event()
        self._held_increments[button] = stop
        Thread(target=self._run_increment, args=(command, stop), daemon=True).start()

    def _run_increment(self, command: Command, stop: Event) -> None:
        step = command.scale / INCREMENT_UPDATE_RATE
        lower = command.metadata.get(CmdMeta.MIN_VALUE, -inf)
        upper = command.metadata.get(CmdMeta.MAX_VALUE, inf)

        # Step before the first wait, so a quick tap still moves the value.
        while self.state.teleop_enabled:
            current = self.state.topic_target_states[command.topic_name]['value']
            new_value = min(max(current + step, lower), upper)
            if new_value != current:
                self.publish_function(command.topic_name, command.message_option, new_value)
            if stop.wait(1 / INCREMENT_UPDATE_RATE):
                return

    def handle_analogue_input(self, input_: str | ControllerInputs | GUIInputs, value: float) -> None:
        """
        Sends the command associated with the control map with the value of the analogue input (e.g., mouse or joystick).
        :param input_: the triggered input.
        :param value: a value from the analogue input.
        """
        if not self.state.teleop_enabled: return
        command: Command = self.state.control_map.get(input_)
        if command is None:
            return

        # Raw input is compared with the previous raw input; post-processed values are scaled, so comparing them
        # with raw input would drop real changes. A return to zero is always sent so a stop can't be filtered out.
        prev_value = self.previous_analogue_values.get(input_)
        is_new_zero = value == 0 and prev_value != 0
        if prev_value is not None and not is_new_zero and abs(value - prev_value) < self.minimum_analogue_change:
            return

        self.previous_analogue_values[input_] = value
        self.publish_function(command.topic_name, command.message_option, self._post_process_value(value, command))

    def _post_process_value(self, value: float, command: Command):
        # Apply motor factors depending on command metadata
        if command.metadata.get(CmdMeta.IS_DRIVE_MOTOR_COMMAND):
            value *= self.state.motor_drive_button_factor
        elif command.metadata.get(CmdMeta.IS_STEER_MOTOR_COMMAND):
            value *= self.state.motor_steer_button_factor
        elif command.metadata.get(CmdMeta.IS_DRUM_MOTOR_COMMAND):
            value *= self.state.motor_drum_button_factor

        return value * command.scale
