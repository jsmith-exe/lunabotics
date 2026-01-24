"""
May be useful for using a controller with macOS (or Linux)
"""


# import hid
#
# # for device in hid.enumerate():
# #     print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")
#
# gamepad = hid.device()
# gamepad.open(0x054c, 0x0ce6)
# gamepad.set_nonblocking(True)
#
# while True:
#     report = gamepad.read(64)
#     if report:
#         print(report)


import hid

gamepad = hid.device()
gamepad.open(0x054c, 0x0ce6)
gamepad.set_nonblocking(True)

# By Claude (gen AI)
def parse_dualsense(data):
    if len(data) < 10:
        return

    # Analog sticks (0-255, center at ~127-128)
    left_x = data[1]
    left_y = data[2]
    right_x = data[3]
    right_y = data[4]

    # Triggers (0-255)
    l2 = data[5]
    r2 = data[6]

    # Buttons are in bytes 8-9 as bit flags
    buttons = data[8]
    buttons2 = data[9]

    # D-pad (byte 8, lower 4 bits)
    dpad = buttons & 0x0F
    dpad_map = {
        0: "Up", 1: "Up-Right", 2: "Right", 3: "Down-Right",
        4: "Down", 5: "Down-Left", 6: "Left", 7: "Up-Left",
        8: "Center"
    }

    # Face buttons (byte 8, upper 4 bits)
    square = bool(buttons & 0x10)
    cross = bool(buttons & 0x20)
    circle = bool(buttons & 0x40)
    triangle = bool(buttons & 0x80)

    # Shoulder buttons (byte 9)
    l1 = bool(buttons2 & 0x01)
    r1 = bool(buttons2 & 0x02)
    l2_button = bool(buttons2 & 0x04)
    r2_button = bool(buttons2 & 0x08)
    share = bool(buttons2 & 0x10)
    options = bool(buttons2 & 0x20)
    l3 = bool(buttons2 & 0x40)  # Left stick click
    r3 = bool(buttons2 & 0x80)  # Right stick click

    return {
        'left_stick': (left_x, left_y),
        'right_stick': (right_x, right_y),
        'l2': l2,
        'r2': r2,
        'dpad': dpad_map.get(dpad, "Unknown"),
        'square': square,
        'cross': cross,
        'circle': circle,
        'triangle': triangle,
        'l1': l1,
        'r1': r1,
        'share': share,
        'options': options,
        'l3': l3,
        'r3': r3
    }

while True:
    report = gamepad.read(64)
    if report:
        parsed = parse_dualsense(report)
        print(parsed)
        continue
        if parsed:
            # Print only when buttons are pressed or sticks moved significantly
            if any([v for k, v in parsed.items() if isinstance(v, bool) and v]) or \
                    abs(parsed['left_stick'][0] - 128) > 10 or \
                    abs(parsed['right_stick'][0] - 128) > 10:
                print(parsed)