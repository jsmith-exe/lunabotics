from collections.abc import Callable
from tkinter import font, ttk

from ttkbootstrap import Style, LabeledScale

from ..constants import DEFAULT_MOTOR_DRIVE_BUTTON_FACTOR, DEFAULT_MOTOR_STEER_BUTTON_FACTOR, \
    DEFAULT_MOTOR_DRUM_BUTTON_FACTOR, GUIInputs
from ..base_station_state import BaseStationState
from ..controllers.base_controller import BaseController
from ..controllers.tkinter_keyboard_controller import TkinterKeyboardController

DANGER_COLOR = "#ff1e39"
UI_SCALE = 2.0  # Match this to your OS display-scaling percentage (e.g. 2.0 for 200%).

class TeleopWindow:
    def __init__(self, base_station_state: BaseStationState, publish_function: Callable, canbus_config: dict):
        self.base_station_state = base_station_state

        self.style = Style(themename='cyborg')
        self.root = self.style.master
        self.root.title("Rover Teleop")
        self.root.resizable(False, False)
        base_scaling = self.root.tk.call('tk', 'scaling')
        self.root.tk.call('tk', 'scaling', base_scaling * UI_SCALE)

        self.slider_controller = BaseController(publish_function, base_station_state, 0.002)
        self.keyboard_controller = TkinterKeyboardController(publish_function, base_station_state, self.root)

        x, y = 0, 0
        self.root.geometry(f"{int(400 * UI_SCALE)}x{int(280 * UI_SCALE)}+{x}+{y}")

        bold_font = font.Font(family="Helvetica", size=22, weight="bold")

        self.message_label = ttk.Label(self.root, text="", font=bold_font)
        self.message_label.pack(pady=(20 * UI_SCALE, 15 * UI_SCALE))

        self.toggle_button = ttk.Button(self.root, text="Disable", command=self.toggle, width=12)
        self.toggle_button.pack(pady=(0, 20 * UI_SCALE))

        sliders_frame = ttk.Frame(self.root)
        sliders_frame.pack(pady=(0, 10 * UI_SCALE))

        make_labeled_slider("Drive Throttle", sliders_frame, 2,
                            lambda value: self.base_station_state.set_motor_drive_button_factor(float(value)),
                            {"value": DEFAULT_MOTOR_DRIVE_BUTTON_FACTOR, "from_": 0, "to": 1},
                            {"side": "left", "padx": 5 * UI_SCALE})

        make_labeled_slider("Steer Throttle", sliders_frame, 2,
                            lambda value: self.base_station_state.set_motor_steer_button_factor(float(value)),
                            {"value": DEFAULT_MOTOR_STEER_BUTTON_FACTOR, "from_": 0, "to": 1},
                            {"side": "left", "padx": 5 * UI_SCALE})

        make_labeled_slider("Drum Throttle", sliders_frame, 2,
                            lambda value: self.base_station_state.set_motor_drum_button_factor(float(value)),
                            {"value": DEFAULT_MOTOR_DRUM_BUTTON_FACTOR, "from_": 0, "to": 1},
                            {"side": "left", "padx": 5 * UI_SCALE})

        drum_config = canbus_config['drum']
        make_labeled_slider("Drum Lift", self.root, 0,
                            lambda value: self.slider_controller.handle_analogue_input(GUIInputs.DRUM_HEIGHT_SLIDER, float(value) / 1000),
                            {"from_": drum_config['min_lift_mm'], "to": drum_config['max_lift_mm'], "value": drum_config['default_lift_mm']},
                            {}, 2)

        self._flash_after_id = None
        self.showing_danger = False

        self.enable()

    def enable(self):
        self.base_station_state.teleop_enabled = True

        self.root.attributes("-topmost", True)
        self.root.focus_force()

        self.message_label.config(
            text="⚠ Teleoperation active",
        )

        self.toggle_button.config(
            text="Disable",
        )

        self._schedule_flash()

    def disable(self):
        self.base_station_state.teleop_enabled = False

        self.root.attributes("-topmost", False)

        self.message_label.config(
            text="✓ Teleoperation disabled",
        )

        self.toggle_button.config(
            text="Enable",
        )

        self.message_label.configure(foreground="")
        self._cancel_flash()

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

    def _schedule_flash(self):
        self.flash_message()
        self._flash_after_id = self.root.after(700, self._schedule_flash)

    def _cancel_flash(self):
        if self._flash_after_id is not None:
            self.root.after_cancel(self._flash_after_id)
            self._flash_after_id = None

    def run(self):
        self.root.mainloop()


def make_labeled_slider(label_text: str, parent, dp: int,
                        on_change: Callable | None = None,
                        slider_kwargs: dict | None = None,
                        packing_opts: dict | None = None,
                        width: int = 1):
    if packing_opts is None: packing_opts = {}
    if slider_kwargs is None: slider_kwargs = {}

    frame = ttk.Frame(parent)
    frame.pack(**packing_opts)

    label = ttk.Label(frame, text=label_text)
    label.pack(pady=(0, 5 * UI_SCALE))

    slider = LabeledScale(frame, **slider_kwargs)
    slider.pack()
    slider.scale.configure(length=int(100 * UI_SCALE * width))

    def handle_move(value):
        rounded = round(float(value), dp)
        slider.value = rounded
        if on_change is not None:
            on_change(rounded)

    slider.scale.configure(command=handle_move)

    return slider


def open_teleop_window(*args):
    window = TeleopWindow(*args)
    window.run()


if __name__ == "__main__":
    state = BaseStationState()
    open_teleop_window(state)
