import tkinter as tk
from tkinter import font

def show_warning(
        bg: str = "#cc8426",
        fg: str = "#000000",
        width: int = 480,
        height: int = 90,
):
    root = tk.Tk()
    root.title("TeleOp Warning")
    root.resizable(False, False)
    root.overrideredirect(True) # no window decorations (pure floating panel)
    root.attributes("-topmost", True) # Stay above ALL other windows
    root.configure(bg=bg)

    # Centre
    root.update_idletasks()
    x, y = 0, 0
    root.geometry(f"{width}x{height}+{x}+{y}")

    bold_font  = font.Font(family="Helvetica", size=22, weight="bold")
    tk.Label(root, text="⚠ Teleoperation active", bg=bg, fg=fg, font=bold_font).pack(pady=(28, 6))

    root.mainloop()

if __name__ == '__main__':
    show_warning()
