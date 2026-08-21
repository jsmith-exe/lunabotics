import threading
import tkinter as tk
from tkinter import font, ttk
import sv_ttk

from ..controls.controllers.base_station_state import BaseStationState

DANGER_COLOR = "#ff6239"

def set_interval(func, interval_seconds: float):
    """
    Recreates setInterval functionality.
    Returns a function to stop the interval.
    """
    stop_event = threading.Event()

    def worker():
        while not stop_event.is_set():
            stop_event.wait(interval_seconds)
            if not stop_event.is_set():
                func()

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()

    def stop():
        print('stopping')
        stop_event.set()
    return stop


class TeleopWindow:
    def __init__(self, base_station_state):
        self.base_station_state = base_station_state # todo replace with state

        self.root = tk.Tk()
        self.root.title("Rover Teleop")
        self.root.resizable(False, False)

        x, y = 0, 0
        self.root.geometry(f"{480}x{120}+{x}+{y}")

        bold_font = font.Font(family="Helvetica", size=22, weight="bold")

        self.message_label = ttk.Label(self.root, text="", font=bold_font)
        self.message_label.pack(pady=(15, 5))

        self.toggle_button = ttk.Button(self.root, text="Disable", command=self.toggle, width=12)
        self.toggle_button.pack(pady=(0, 10))

        self.stop_flashing_interval = lambda : None  # Placeholder for the flashing interval function
        self.showing_danger = False

        self.enable()

    def enable(self):
        self.base_station_state.teleop_enabled = True

        self.root.attributes("-topmost", True)
        self.root.overrideredirect(True)

        self.message_label.config(
            text="⚠ Teleoperation active",
        )

        self.toggle_button.config(
            text="Disable",
        )

        self.stop_flashing_interval = set_interval(self.flash_message, 0.7)

    def disable(self):
        self.base_station_state.teleop_enabled = False

        self.root.attributes("-topmost", False)
        self.root.overrideredirect(False)

        self.message_label.config(
            text="✓ Teleoperation disabled",
        )

        self.toggle_button.config(
            text="Enable",
        )

        self.message_label.configure(foreground="")
        self.stop_flashing_interval()

    def toggle(self):
        if self.base_station_state.teleop_enabled:
            self.disable()
        else:
            self.enable()

    def flash_message(self):
        self.showing_danger = not self.showing_danger
        if self.showing_danger:
            self.message_label.configure(foreground=DANGER_COLOR)
        else:
            self.message_label.configure(foreground="")

    def run(self):
        sv_ttk.set_theme("dark")
        self.root.mainloop()

def open_teleop_window(base_station_state):
    window = TeleopWindow(base_station_state)
    window.run()

if __name__ == "__main__":
    state = BaseStationState()
    open_teleop_window(state)
