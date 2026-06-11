import tkinter as tk
from tkinter import font

from ..controls.controllers.base_station_state import BaseStationState

class TeleopWindow:
    def __init__(self, base_station_state):
        self.enabled_bg = "#cc8426"
        self.disabled_bg = "#2e8b57"
        self.btn_enabled_bg = "#f16c15"
        self.btn_disabled_bg = "#4caf50"

        self.fg = "#000000"
        self.base_station_state = base_station_state # todo replace with state

        self.root = tk.Tk()
        self.root.title("Rover Teleop")
        self.root.resizable(False, False)

        x, y = 0, 0
        self.root.geometry(f"{480}x{120}+{x}+{y}")

        bold_font = font.Font(family="Helvetica", size=22, weight="bold")

        self.message_label = tk.Label(self.root, text="", font=bold_font)
        self.message_label.pack(pady=(15, 5))

        self.toggle_button = tk.Button(self.root, text="Disable", command=self.toggle, width=12)
        self.toggle_button.pack(pady=(0, 10))

        self.enable()

    def enable(self):
        self.base_station_state.teleop_enabled = True

        self.root.attributes("-topmost", True)
        self.root.overrideredirect(True)
        self.root.configure(bg=self.enabled_bg)

        self.message_label.config(
            text="⚠ Teleoperation active",
            bg=self.enabled_bg,
        )

        self.toggle_button.config(
            text="Disable",
            bg=self.btn_enabled_bg,
        )

    def disable(self):
        self.base_station_state.teleop_enabled = False

        self.root.attributes("-topmost", False)
        self.root.overrideredirect(False)
        self.root.configure(bg=self.disabled_bg)

        self.message_label.config(
            text="✓ Teleoperation disabled",
            bg=self.disabled_bg,
        )

        self.toggle_button.config(
            text="Enable",
            bg=self.btn_disabled_bg,
        )

    def toggle(self):
        if self.base_station_state.teleop_enabled:
            self.disable()
        else:
            self.enable()

    def run(self):
        self.root.mainloop()

def open_teleop_window(base_station_state):
    warning = TeleopWindow(base_station_state)
    warning.run()

if __name__ == "__main__":
    state = BaseStationState()
    open_teleop_window(state)
