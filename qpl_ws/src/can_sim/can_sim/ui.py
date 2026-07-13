"""
FoilBus visualiser — live view of simulated SPARK MAX state.

Usage: pass the same can_id_to_state_mapping dict used by CANBusSim,
along with a layout mapping that assigns each CAN ID to a position.

Layout positions:
    "left"   — left column  (e.g. left-side drive motors)
    "right"  — right column (e.g. right-side drive motors)
    "centre" — centre column (drum, actuators)

Example:

    layout = {
        1: ("left",   "Front Left"),
        2: ("right",  "Front Right"),
        3: ("left",   "Rear Left"),
        4: ("right",  "Rear Right"),
        5: ("centre", "Left Actuator"),
        6: ("centre", "Right Actuator"),
        7: ("centre", "Drum"),
    }

    window = SimWindow(can_setup, layout)
    window.start()   # blocks — call from main thread
"""

import tkinter as tk
import threading
import time
from .core.classes import Motor, Actuator

# --- Palette ---
BG          = "#1a1a1a"   # window background
PANEL_BG    = "#242424"   # device card background
BORDER      = "#333333"   # card border
TEXT        = "#e0e0e0"   # primary text
DIM         = "#666666"   # label / secondary text
ACCENT_POS  = "#4caf82"   # positive RPM / extended
ACCENT_NEG  = "#e05c5c"   # negative RPM / retracted
ACCENT_IDLE = "#555555"   # zero / stopped
BAR_BG      = "#333333"   # progress bar track

POLL_MS     = 50          # UI refresh interval (milliseconds)

FONT_LABEL  = ("Courier New", 9)
FONT_VALUE  = ("Courier New", 13, "bold")
FONT_TITLE  = ("Courier New", 10, "bold")
FONT_HEADER = ("Courier New", 11, "bold")


def _bar_colour(value: float) -> str:
    """Return accent colour based on sign of value."""
    if value > 0.01:
        return ACCENT_POS
    if value < -0.01:
        return ACCENT_NEG
    return ACCENT_IDLE


class MotorCard(tk.Frame):
    """
    Displays live state for one Motor: RPM bar and position counter.
    The bar fills left-to-right for positive RPM, right-to-left for negative.
    """

    BAR_W = 160
    BAR_H = 14

    def __init__(self, parent, label: str, motor: Motor):
        super().__init__(parent, bg=PANEL_BG, bd=0, highlightthickness=1,
                         highlightbackground=BORDER)
        self._motor = motor
        self._build(label)

    def _build(self, label: str):
        tk.Label(self, text=label, font=FONT_TITLE, bg=PANEL_BG,
                 fg=TEXT).pack(anchor="w", padx=8, pady=(8, 2))

        tk.Label(self, text=f"CAN ID {self._motor.can_id}",
                 font=FONT_LABEL, bg=PANEL_BG, fg=DIM).pack(anchor="w", padx=8)

        # RPM bar canvas
        self._canvas = tk.Canvas(self, width=self.BAR_W, height=self.BAR_H,
                                 bg=PANEL_BG, bd=0, highlightthickness=0)
        self._canvas.pack(padx=8, pady=4)
        self._track = self._canvas.create_rectangle(
            0, 0, self.BAR_W, self.BAR_H, fill=BAR_BG, outline="")
        self._bar = self._canvas.create_rectangle(
            0, 0, 0, self.BAR_H, fill=ACCENT_IDLE, outline="")

        # RPM value
        self._rpm_var = tk.StringVar(value="0 RPM")
        tk.Label(self, textvariable=self._rpm_var, font=FONT_VALUE,
                 bg=PANEL_BG, fg=TEXT).pack(anchor="w", padx=8)

        # Position
        tk.Label(self, text="position", font=FONT_LABEL,
                 bg=PANEL_BG, fg=DIM).pack(anchor="w", padx=8, pady=(4, 0))
        self._pos_var = tk.StringVar(value="0.00 rev")
        tk.Label(self, textvariable=self._pos_var, font=FONT_LABEL,
                 bg=PANEL_BG, fg=TEXT).pack(anchor="w", padx=8, pady=(0, 8))

    def refresh(self):
        rpm = self._motor.measured_rpm
        pos = self._motor.position_rotations
        max_rpm = self._motor.max_rpm

        # Update bar — centre is 0, fill direction depends on sign
        fraction = max(-1.0, min(1.0, rpm / max_rpm))
        mid = self.BAR_W // 2
        fill_px = int(abs(fraction) * mid)
        if fraction >= 0:
            x0, x1 = mid, mid + fill_px
        else:
            x0, x1 = mid - fill_px, mid

        self._canvas.coords(self._bar, x0, 0, x1, self.BAR_H)
        self._canvas.itemconfig(self._bar, fill=_bar_colour(rpm))

        self._rpm_var.set(f"{rpm:+.0f} RPM")
        self._pos_var.set(f"{pos:.2f} rev")


class ActuatorCard(tk.Frame):
    """
    Displays live state for one Actuator: position bar (0-1) and voltage.
    """

    BAR_W = 160
    BAR_H = 14

    def __init__(self, parent, label: str, actuator: Actuator):
        super().__init__(parent, bg=PANEL_BG, bd=0, highlightthickness=1,
                         highlightbackground=BORDER)
        self._actuator = actuator
        self._build(label)

    def _build(self, label: str):
        tk.Label(self, text=label, font=FONT_TITLE, bg=PANEL_BG,
                 fg=TEXT).pack(anchor="w", padx=8, pady=(8, 2))

        tk.Label(self, text=f"CAN ID {self._actuator.can_id}",
                 font=FONT_LABEL, bg=PANEL_BG, fg=DIM).pack(anchor="w", padx=8)

        # Position bar — always fills left to right, 0 to 1
        self._canvas = tk.Canvas(self, width=self.BAR_W, height=self.BAR_H,
                                 bg=PANEL_BG, bd=0, highlightthickness=0)
        self._canvas.pack(padx=8, pady=4)
        self._canvas.create_rectangle(0, 0, self.BAR_W, self.BAR_H,
                                      fill=BAR_BG, outline="")
        self._bar = self._canvas.create_rectangle(0, 0, 0, self.BAR_H,
                                                  fill=ACCENT_POS, outline="")

        self._pos_var = tk.StringVar(value="0.50")
        tk.Label(self, textvariable=self._pos_var, font=FONT_VALUE,
                 bg=PANEL_BG, fg=TEXT).pack(anchor="w", padx=8)

        tk.Label(self, text="duty", font=FONT_LABEL,
                 bg=PANEL_BG, fg=DIM).pack(anchor="w", padx=8, pady=(4, 0))
        self._duty_var = tk.StringVar(value="0.00")
        tk.Label(self, textvariable=self._duty_var, font=FONT_LABEL,
                 bg=PANEL_BG, fg=TEXT).pack(anchor="w", padx=8, pady=(0, 8))

    def refresh(self):
        pos  = self._actuator.position
        duty = self._actuator.commanded_duty

        fill_px = int(pos * self.BAR_W)
        self._canvas.coords(self._bar, 0, 0, fill_px, self.BAR_H)
        colour = ACCENT_POS if duty > 0.01 else (ACCENT_NEG if duty < -0.01 else ACCENT_IDLE)
        self._canvas.itemconfig(self._bar, fill=colour)

        self._pos_var.set(f"{pos:.2f}")
        self._duty_var.set(f"{duty:+.2f}")


class SimWindow:
    """
    Main window. Accepts the same can_id_to_state_mapping as CANBusSim,
    plus a layout dict that assigns each CAN ID a column and label:

        layout = {can_id: (column, label), ...}
        column is one of "left", "centre", "right"
    """

    def __init__(self, can_id_to_state_mapping: dict, layout: dict):
        self._devices = can_id_to_state_mapping
        self._layout  = layout
        self._cards   = {}   # can_id -> card widget

        self._root = tk.Tk()
        self._root.title("FoilBus — SPARK MAX Simulator")
        self._root.configure(bg=BG)
        self._root.resizable(False, False)

        self._build()

    def _build(self):
        # Three columns: left, centre, right
        columns = {
            "left":   tk.Frame(self._root, bg=BG),
            "centre": tk.Frame(self._root, bg=BG),
            "right":  tk.Frame(self._root, bg=BG),
        }

        for col_name, frame in columns.items():
            header_text = {"left": "LEFT", "centre": "CENTRE", "right": "RIGHT"}[col_name]
            tk.Label(frame, text=header_text, font=FONT_HEADER,
                     bg=BG, fg=DIM).pack(pady=(12, 6))
            frame.pack(side="left", fill="both", expand=True, padx=12, pady=8)

        # Create a card for each device according to layout
        for can_id, device in self._devices.items():
            col_name, label = self._layout.get(can_id, ("centre", f"Device {can_id}"))
            parent = columns[col_name]

            if isinstance(device, Motor):
                card = MotorCard(parent, label, device)
            elif isinstance(device, Actuator):
                card = ActuatorCard(parent, label, device)
            else:
                continue

            card.pack(fill="x", pady=4)
            self._cards[can_id] = card

    def _poll(self):
        """Refresh all cards from current device state, then reschedule."""
        for can_id, card in self._cards.items():
            card.refresh()
        self._root.after(POLL_MS, self._poll)

    def start(self):
        """Start polling and enter the Tk main loop. Blocks until window closes."""
        self._root.after(POLL_MS, self._poll)
        self._root.mainloop()
