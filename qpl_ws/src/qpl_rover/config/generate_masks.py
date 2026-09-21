import os
import imageio.v2 as imageio
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# ----------------------------------------------------------------
# ARENA AND MAP CONFIGURATION
# ----------------------------------------------------------------
# Ensure these match your actual SLAM map resolution and dimensions!
RESOLUTION = 0.05  # meters per pixel (matches your Nav2 YAML configuration)

# Real-world arena bounds (meters)
ARENA_WIDTH_X = 4.4
ARENA_LENGTH_Y = 7.9

# Buffer/Padding: How much extra "outer world" space to draw in meters
# This ensures the image has a safe black border surrounding the white arena box.
PADDING_M = 1.0

# Total dimensions of the image in meters
TOTAL_WIDTH_X = ARENA_WIDTH_X + (2 * PADDING_M)
TOTAL_LENGTH_Y = ARENA_LENGTH_Y + (2 * PADDING_M)

# Convert total metric dimensions to pixels
PIX_X = int(np.ceil(TOTAL_WIDTH_X / RESOLUTION))
PIX_Y = int(np.ceil(TOTAL_LENGTH_Y / RESOLUTION))

# ----------------------------------------------------------------
# GENERATE MASK ARRAYS
# ----------------------------------------------------------------
# 1. Initialize the entire world as LETHAL OBSTACLE / KEEPOUT (Solid Black = 0)
mask = np.zeros((PIX_Y, PIX_X), dtype=np.uint8)

# 2. Calculate the pixel boundaries of the valid, safe driving zone
start_x_px = int(PADDING_M / RESOLUTION)
end_x_px = start_x_px + int(ARENA_WIDTH_X / RESOLUTION)

start_y_px = int(PADDING_M / RESOLUTION)
end_y_px = start_y_px + int(ARENA_LENGTH_Y / RESOLUTION)

# 3. Paint the inside of the arena as FREE SPACE (Solid White = 255)
mask[start_y_px:end_y_px, start_x_px:end_x_px] = 255

# ----------------------------------------------------------------
# CALCULATE THE CALIBRATION FILE (YAML ORIGIN)
# ----------------------------------------------------------------
# Nav2 needs to know where the bottom-left corner of this image sits relative to map's (0,0) origin.
# Since map (0,0) is the bottom-left corner of the white arena box,
# the bottom-left corner of the whole image sits exactly at negative padding.
origin_x = -PADDING_M
origin_y = -PADDING_M

# ----------------------------------------------------------------
# EXPORT FILES
# ----------------------------------------------------------------
image_path = os.path.join(BASE_DIR, "keepout_mask.pgm")
imageio.imwrite(image_path, mask)

yaml_path = os.path.join(BASE_DIR, "keepout_mask.yaml")
yaml_content = f"""image: keepout_mask.pgm
resolution: {RESOLUTION}
origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""

with open(yaml_path, "w") as f:
    f.write(yaml_content)

print(f"Generated keep-out mask size: {PIX_X}x{PIX_Y} pixels")
print(f"Calculated YAML origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]")