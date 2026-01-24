"""
Controller class for desktop environments using keyboard and mouse inputs.
Helpful documentation:
- pynput docs: https://pynput.readthedocs.io/en/latest/index.html
"""

from pynput import keyboard, mouse

from .base_controller import BaseController
from .base_station_state import BaseStationState

class DesktopController(BaseController):
    """ Translates keyboard and mouse inputs to commands. """
    def __init__(self, publish_function, state):
        super().__init__(publish_function, state)
        self.key_states = dict()

        # Setup listeners
        self.keyboard_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.mouse_listener = mouse.Listener(on_move=self.on_mouse_move, on_click=self.on_mouse_click, on_scroll=self.on_mouse_scroll)
        self.keyboard_listener.start()
        self.mouse_listener.start()

    def on_press(self, key_object):
        """ Handles a key press event, taking in a key object from pynput. """
        self.handle_key(key_object, True)

    def on_release(self, key_object):
        """ Handles a key release event, taking in a key object from pynput. """
        self.handle_key(key_object, False)

    def handle_key(self, key_object, pressed: bool):
        """
        Generalized key handler for both press and release events.
        :param key_object: a key object from pynput.
        :param pressed: whether the key event is a press (True) or release (False).
        """
        control_map = self.state.desktop_control_map

        # Some keys use 'char' (a, b), others use 'name' (shift, ctrl).
        # Get the appropriate attribute as a generalised name.
        key_name = getattr(key_object, 'char', None) or getattr(key_object, 'name', None)

        if (key_name not in control_map
            # Avoid duplicate state changes; these can occur if a key is held down.
            or self.key_states.get(key_name) == pressed):
            return

        self.key_states[key_name] = pressed
        self.handle_button(key_name, pressed, control_map)

    def on_mouse_move(self, x, y):
        pass

    def on_mouse_click(self, x, y, button, pressed):
        pass

    def on_mouse_scroll(self, x, y, dx, dy):
        pass


if __name__ == '__main__':
    DesktopController(print, BaseStationState())
    from time import sleep
    sleep(1000)
