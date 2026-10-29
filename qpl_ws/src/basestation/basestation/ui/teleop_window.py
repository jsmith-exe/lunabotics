from collections.abc import Callable
from tkinter import font, ttk, PhotoImage
from os import environ

from ttkbootstrap import Style, LabeledScale

from ..constants import DEFAULT_MOTOR_DRIVE_BUTTON_FACTOR, DEFAULT_MOTOR_STEER_BUTTON_FACTOR, \
    DEFAULT_MOTOR_DRUM_BUTTON_FACTOR, GUIInputs, ICON_PATH, NAV_TOPIC, MessageOptions, DRUM_ROTATION_TOPIC, DRUM_LIFT_TOPIC
from ..base_station_state import BaseStationState
from ..controllers.base_controller import BaseController
from ..controllers.tkinter_keyboard_controller import TkinterKeyboardController

DANGER_COLOR = "#ff1e39"
UI_SCALE = float(environ.get('QPL_TELEOP_WINDOW_SCALE', 1.0))
WINDOW_WIDTH = 300  # Minimum width; the window widens if expanded sections need more room.

class TeleopWindow:
    def __init__(self, base_station_state: BaseStationState, publish_function: Callable, canbus_config: dict,
                 autonomy_command_function: Callable[[str], None],
                 nav_autonomy_command_function: Callable[[str], None]):
        self.base_station_state = base_station_state
        self.publish_function = publish_function
        self.canbus_config = canbus_config
        self.autonomy_command_function = autonomy_command_function
        self.nav_autonomy_command_function = nav_autonomy_command_function

        self.style = Style(themename='cyborg')
        self.root = self.style.master
        self.root.title("Rover Teleop")
        self.root.resizable(False, False)
        base_scaling = self.root.tk.call('tk', 'scaling')
        self.root.tk.call('tk', 'scaling', base_scaling * UI_SCALE)

        # Icon
        photo = PhotoImage(file=ICON_PATH)
        self.root.wm_iconphoto(False, photo)

        # Default exception hanlder prints error; in the event of KeyboardInterrupt, ignore error and close window.
        def report_callback_exception(exc, val, tb):
            if issubclass(exc, KeyboardInterrupt):
                self.root.destroy()
            else:
                self.root.__class__.report_callback_exception(self.root, exc, val, tb)
        self.root.report_callback_exception = report_callback_exception

        self.slider_controller = BaseController(publish_function, base_station_state, 0.001)
        self.keyboard_controller = TkinterKeyboardController(publish_function, base_station_state, self.root)

        self._flash_after_id = None
        self.showing_danger = False

        self.add_teleop_toggle()
        self.add_drum_lift_slider()
        self.add_throttle_section()
        self.add_autonomy_section()
        self.add_nav_autonomy_section()

        self.disable()
        self._sync_drum_lift_slider() # After enable(), which sets the lift target from the slider

        # Make sure the initial size fits everything, including the status message set by enable().
        self.root.update_idletasks()
        height = max(int(260 * UI_SCALE), self.root.winfo_reqheight())
        self.root.geometry(f"{fit_window_width(self.root)}x{height}+0+0")

    def add_teleop_toggle(self):
        bold_font = font.Font(family="Helvetica", size=18, weight="bold")
        self.message_label = ttk.Label(self.root, text="", font=bold_font)
        self.message_label.pack(pady=(20 * UI_SCALE, 15 * UI_SCALE))
        self.toggle_button = ttk.Button(self.root, text="Disable", command=self.toggle, width=12)
        self.toggle_button.pack(pady=(0, 20 * UI_SCALE))

    def add_throttle_section(self):
        self.throttle_section = CollapsibleSection(self.root, "Throttle")
        sliders_frame = self.throttle_section.content

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

    def add_drum_lift_slider(self):
        drum_config = self.canbus_config['drum']
        self.drum_lift_slider = make_labeled_slider("Drum Lift", self.root, 0,
                            lambda value: self.slider_controller.handle_analogue_input(GUIInputs.DRUM_HEIGHT_SLIDER, float(value) / 1000),
                            {"from_": drum_config['min_lift_mm'], "to": drum_config['max_lift_mm'], "value": drum_config['max_lift_mm']},
                            {}, 2)

    def _sync_drum_lift_slider(self):
        """ Buttons can also move the drum lift, so keep the slider showing the current lift target. """
        # Skipped while disabled, where the target is reset to 0; enable() republishes the slider value.
        if self.base_station_state.teleop_enabled:
            lift_mm = round(self.base_station_state.topic_target_states[DRUM_LIFT_TOPIC]['value'] * 1000, 2)
            if lift_mm != self.drum_lift_slider.value:
                self.drum_lift_slider.value = lift_mm
                # The slider's last sent value is now stale; forget it so the next drag isn't filtered out.
                self.slider_controller.previous_analogue_values.pop(GUIInputs.DRUM_HEIGHT_SLIDER, None)
        self.root.after(100, self._sync_drum_lift_slider)

    def add_autonomy_section(self):
        self.autonomy_section = CollapsibleSection(self.root, "Autonomy")

        buttons = {}
        for text, command in (("Excavate", "EXCAVATE"), ("Deposit", "DEPOSIT")):
            buttons[command] = ttk.Button(self.autonomy_section.content, text=text, width=10,
                                          command=lambda command=command: self.autonomy_command_function(command))
            buttons[command].pack(side="left", padx=5 * UI_SCALE)
        ttk.Button(self.autonomy_section.content, text="Stop", width=10,
                   command=self.stop_all_autonomy).pack(side="left", padx=5 * UI_SCALE)

        # Autonomy can only be started while teleop is disabled; Stop is always available.
        self.autonomy_start_buttons = [buttons["EXCAVATE"], buttons["DEPOSIT"]]

    def add_nav_autonomy_section(self):
        """ Nav2 autonomy, run by qpl_nav. Separate from the blind autonomy above, which is kept as a backup. """
        self.nav_autonomy_section = CollapsibleSection(self.root, "Nav Autonomy")

        # Nav excavate/deposit on the first row; full autonomy and Stop on the second
        nav_row = ttk.Frame(self.nav_autonomy_section.content)
        nav_row.pack()
        auto_row = ttk.Frame(self.nav_autonomy_section.content)
        auto_row.pack(pady=(5 * UI_SCALE, 0))

        # Nav versions drive to the zone's waypoint with Nav2 first; full autonomy loops both until Stop
        for row, text, command in ((nav_row, "Nav Excavate", "NAV_EXCAVATE"), (nav_row, "Nav Deposit", "NAV_DEPOSIT"),
                                   (auto_row, "Full Autonomy", "AUTO")):
            button = ttk.Button(row, text=text,
                                command=lambda command=command: self.nav_autonomy_command_function(command))
            button.pack(side="left", padx=5 * UI_SCALE)
            self.autonomy_start_buttons.append(button)
        ttk.Button(auto_row, text="Stop", width=10,
                   command=self.stop_all_autonomy).pack(side="left", padx=5 * UI_SCALE)

        # Recorder mode: lay a breadcrumb path while driving from the excavation zone to the
        # deposition point. These work while teleop is enabled, since the operator is driving.
        record_row = ttk.Frame(self.nav_autonomy_section.content)
        record_row.pack(pady=(5 * UI_SCALE, 0))
        for text, command in (("Record Start", "RECORD_START"), ("Record Waypoint", "RECORD_WAYPOINT"),
                              ("Record End", "RECORD_END")):
            ttk.Button(record_row, text=text,
                       command=lambda command=command: self.nav_autonomy_command_function(command)
                       ).pack(side="left", padx=5 * UI_SCALE)

        # Drive the recorded path either way, no obstacle avoidance; only while teleop is disabled
        follow_row = ttk.Frame(self.nav_autonomy_section.content)
        follow_row.pack(pady=(5 * UI_SCALE, 0))
        for text, command in (("Follow", "FOLLOW_TO_DEPOSIT"), ("Follow reversed", "FOLLOW_TO_EXCAVATE")):
            button = ttk.Button(follow_row, text=text,
                                command=lambda command=command: self.nav_autonomy_command_function(command))
            button.pack(side="left", padx=5 * UI_SCALE)
            self.autonomy_start_buttons.append(button)

    def stop_all_autonomy(self):
        # Either Stop button stops both autonomy nodes, so the operator can't press the wrong one
        self.autonomy_command_function("STOP")
        self.nav_autonomy_command_function("STOP")

    def enable(self):
        self.base_station_state.teleop_enabled = True
        self.root.attributes("-topmost", True)
        self.root.focus_force()
        self.message_label.config(text="⚠ Teleoperation active")
        self.toggle_button.config(text="Disable")
        self._schedule_flash()
        for button in self.autonomy_start_buttons:
            button.state(["disabled"])

        self.publish_function(DRUM_LIFT_TOPIC, MessageOptions.FLOAT, self.drum_lift_slider.value / 1000)

    def disable(self):
        # Send an explicit wheel stop before going quiet, rather than relying on the rover's command timeout.
        # Publishing is ignored once teleop is disabled, so this must happen first.
        self.base_station_state.reset_topic_states()
        self.publish_function(NAV_TOPIC, MessageOptions.TWIST_LINEAR_X, 0)
        self.publish_function(NAV_TOPIC, MessageOptions.TWIST_ANGULAR_Z, 0)
        self.publish_function(DRUM_ROTATION_TOPIC, MessageOptions.FLOAT, 0)
        self.publish_function(DRUM_LIFT_TOPIC, MessageOptions.FLOAT, 0)

        self.base_station_state.teleop_enabled = False
        self.root.attributes("-topmost", False)
        self.message_label.config(text="✓ Teleoperation disabled")
        self.toggle_button.config(text="Enable")
        self.message_label.configure(foreground="")
        self._cancel_flash()
        for button in self.autonomy_start_buttons:
            button.state(["!disabled"])

    def toggle(self):
        if self.base_station_state.teleop_enabled:
            self.disable()
        else:
            self.enable()

        # The enabled/disabled messages differ in width, so refit the window to the new message.
        self.root.update_idletasks()
        self.root.geometry(f"{fit_window_width(self.root)}x{self.root.winfo_height()}")

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


class CollapsibleSection:
    """ A header button that shows/hides a content frame. Add widgets to `content`. """
    def __init__(self, parent, title: str):
        self.title = title
        self.expanded = False

        # Header and content share a container, so the content opens directly below its own header.
        self.container = ttk.Frame(parent)
        self.container.pack(pady=(10 * UI_SCALE, 0))
        self.toggle_button = ttk.Button(self.container, text=f"▶ {title}", command=self.toggle, width=12)
        self.toggle_button.pack()

        # Only packed while the section is expanded.
        self.content = ttk.Frame(self.container)

    def toggle(self):
        # The window has a fixed size, so grow/shrink its height by however much the required height changes,
        # and fit its width to the content.
        window = self.container.winfo_toplevel()
        window.update_idletasks()
        height_before = window.winfo_reqheight()

        if self.expanded:
            self.content.pack_forget()
            self.toggle_button.config(text=f"▶ {self.title}")
        else:
            self.content.pack(pady=(10 * UI_SCALE, 0))
            self.toggle_button.config(text=f"▼ {self.title}")
        self.expanded = not self.expanded

        window.update_idletasks()
        height_change = window.winfo_reqheight() - height_before
        window.geometry(f"{fit_window_width(window)}x{window.winfo_height() + height_change}")


def fit_window_width(window) -> int:
    """ The default window width, or wider if the currently visible content needs it. """
    return max(int(WINDOW_WIDTH * UI_SCALE), window.winfo_reqwidth())


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
