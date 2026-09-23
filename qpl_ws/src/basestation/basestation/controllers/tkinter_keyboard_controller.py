import tkinter as tk

from .base_controller import BaseController
from ..base_station_state import BaseStationState


class TkinterKeyboardController(BaseController):
    """ Translates keyboard input, captured via Tkinter's own event bindings, to commands. """

    _REPEAT_GUARD_MS = 20

    def __init__(self, publish_function, state: BaseStationState, widget: tk.Misc):
        super().__init__(publish_function, state)
        self.widget = widget
        self.key_states: dict[str, bool] = {}
        # Holding keys spams presses and releases. Handle by scheduling the release process and cancelling it if a press
        # is received too quickly. Keys and their call ID are stored in this dict.
        self._pending_releases: dict[str, str] = {}

        widget.bind("<KeyPress>", self._on_key_press, add="+")
        widget.bind("<KeyRelease>", self._on_key_release, add="+")
        widget.bind("<FocusOut>", self._on_focus_out, add="+")

    def _on_key_press(self, event: tk.Event):
        key_name = event.keysym.lower()

        # Holding keys spams presses and releases; handled by checking if this key was still waiting to be released,
        # if so, cancel the call that processes the release.
        pending_id = self._pending_releases.pop(key_name, None)
        if pending_id is not None: # Key is being held. Cancel the existing call to process release.
            self.widget.after_cancel(pending_id)
            return

        self._handle_key(key_name, True)

    def _on_key_release(self, event: tk.Event):
        key_name = event.keysym.lower()
        # Schedule a call to indicate release; this will be interrupted if a press is received before _REPEAT_GUARD_MS,
        # to ignore key-press-release spam when holding keys.
        self._pending_releases[key_name] = self.widget.after(self._REPEAT_GUARD_MS, self._finish_release, key_name)

    def _finish_release(self, key_name: str):
        self._pending_releases.pop(key_name, None)
        self._handle_key(key_name, False)

    def _on_focus_out(self, _event: tk.Event):
        """ Fail safe: treat every held key as released when focus is lost, since no further
        KeyRelease events will arrive for it while the widget isn't focused. """
        for after_id in self._pending_releases.values():
            self.widget.after_cancel(after_id)
        self._pending_releases.clear()

        for key_name, pressed in list(self.key_states.items()):
            if pressed:
                self.key_states[key_name] = False
                self.handle_button(key_name, False)

    def _handle_key(self, key_name: str, pressed: bool):
        if self.key_states.get(key_name) == pressed:
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
