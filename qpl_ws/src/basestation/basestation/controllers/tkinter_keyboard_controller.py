import tkinter as tk

from .base_controller import BaseController
from ..base_station_state import BaseStationState


class TkinterKeyboardController(BaseController):
    """ Translates keyboard input, captured via Tkinter's own event bindings, to commands. """
    def __init__(self, publish_function, state: BaseStationState, widget: tk.Misc):
        super().__init__(publish_function, state)
        self.widget = widget
        self.key_states: dict[str, bool] = {}

        widget.bind("<KeyPress>", self._on_key_press, add="+")
        widget.bind("<KeyRelease>", self._on_key_release, add="+")
        widget.bind("<FocusOut>", self._on_focus_out, add="+")

    def _on_key_press(self, event: tk.Event):
        self._handle_key(event, True)

    def _on_key_release(self, event: tk.Event):
        self._handle_key(event, False)

    def _on_focus_out(self, _event: tk.Event):
        """ Fail safe: treat every held key as released when focus is lost, since no further
        KeyRelease events will arrive for it while the widget isn't focused. """
        for key_name, pressed in list(self.key_states.items()):
            if pressed:
                self.key_states[key_name] = False
                self.handle_button(key_name, False)

    def _handle_key(self, event: tk.Event, pressed: bool):
        # Tkinter's keysym reflects shift state for letters (e.g. 'W' vs 'w'); lowercase so
        # a held key's press/release pair always matches the same name in the control map.
        key_name = event.keysym.lower()

        if self.key_states.get(key_name) == pressed:
            # Avoid duplicate state changes; these occur while a key is held (OS key repeat).
            return

        self.key_states[key_name] = pressed
        self.handle_button(key_name, pressed)


if __name__ == '__main__':
    root = tk.Tk()
    root.title("TkinterKeyboardController demo")
    tk.Label(root, text="Click this window, then press keys (see terminal). Esc to quit.").pack(padx=20, pady=20)
    root.bind("<Escape>", lambda _event: root.destroy())

    TkinterKeyboardController(print, BaseStationState(), root)
    root.mainloop()
