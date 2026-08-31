import threading
from collections.abc import Callable
from tkinter import font, ttk

from ttkbootstrap import Style, LabeledScale

from ..constants import DEFAULT_MOTOR_DRIVE_BUTTON_FACTOR, DEFAULT_MOTOR_STEER_BUTTON_FACTOR, \
    DEFAULT_MOTOR_DRUM_BUTTON_FACTOR, GUIInputs
from ..controls.controllers.base_station_state import BaseStationState
from ..controls.controllers.base_controller import BaseController

DANGER_COLOR = "#ff1e39"

def make_labeled_slider(label_text: str, parent, dp: int, on_change: Callable | None = None,
                        slider_kwargs: dict | None = None, packing_opts: dict | None = None):
    if packing_opts is None: packing_opts = {}
    if slider_kwargs is None: slider_kwargs = {}

    frame = ttk.Frame(parent)
    frame.pack(**packing_opts)

    label = ttk.Label(frame, text=label_text)
    label.pack(pady=(0, 5))

    slider = LabeledScale(frame, **slider_kwargs)
    slider.pack()

    def handle_move(value):
        rounded = round(float(value), dp)
        slider.value = rounded
        if on_change is not None:
            on_change(rounded)

    slider.scale.configure(command=handle_move)

    return slider

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
        stop_event.set()
    return stop


class TeleopWindow:
    def __init__(self, base_station_state: BaseStationState, publish_function: Callable):
        self.base_station_state = base_station_state
        self.controller = BaseController(publish_function, base_station_state)

        self.style = Style(themename='cyborg')
        self.root = self.style.master
        self.root.title("Rover Teleop")
        # self.root.resizable(False, False)

        x, y = 0, 0
        self.root.geometry(f"{480}x{240}+{x}+{y}")

        bold_font = font.Font(family="Helvetica", size=22, weight="bold")

        self.message_label = ttk.Label(self.root, text="", font=bold_font)
        self.message_label.pack(pady=(15, 5))

        self.toggle_button = ttk.Button(self.root, text="Disable", command=self.toggle, width=12)
        self.toggle_button.pack(pady=(0, 10))

        sliders_frame = ttk.Frame(self.root)
        sliders_frame.pack(pady=(0, 10))

        make_labeled_slider("Drive Throttle", sliders_frame, 2,
                            lambda value: self.base_station_state.set_motor_drive_button_factor(float(value)),
                            {"value": DEFAULT_MOTOR_DRIVE_BUTTON_FACTOR, "from_": 0, "to": 1},
                            {"side": "left", "padx": 5})

        make_labeled_slider("Steer Throttle", sliders_frame, 2,
                            lambda value: self.base_station_state.set_motor_steer_button_factor(float(value)),
                            {"value": DEFAULT_MOTOR_STEER_BUTTON_FACTOR, "from_": 0, "to": 1},
                            {"side": "left", "padx": 5})

        make_labeled_slider("Drum Throttle", sliders_frame, 2,
                            lambda value: self.base_station_state.set_motor_drum_button_factor(float(value)),
                            {"value": DEFAULT_MOTOR_DRUM_BUTTON_FACTOR, "from_": 0, "to": 1},
                            {"side": "left", "padx": 5})

        make_labeled_slider("Drum Lift", self.root, 0,
                            lambda value: self.controller.handle_analogue_input(GUIInputs.DRUM_HEIGHT_SLIDER, float(value)),
                            {"from_": 22.6, "to": 228.0, "value": 100})

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
        self.root.mainloop()

def open_teleop_window(*args):
    window = TeleopWindow(*args)
    window.run()

if __name__ == "__main__":
    state = BaseStationState()
    open_teleop_window(state)
