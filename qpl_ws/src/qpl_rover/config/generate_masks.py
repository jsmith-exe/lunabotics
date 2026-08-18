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
# UCF championship arena (guidebook Figure 2; see arena_ucf.world for the
# frame convention — map origin is the SW interior corner).
ARENA_WIDTH_X = 9.14
ARENA_LENGTH_Y = 8.10

# Buffer/Padding: How much extra "outer world" space to draw in meters
# This ensures the image has a safe black border surrounding the white arena box.
PADDING_M = 1.0

# Clearance: how far the rover's CENTRE is kept back from the true arena wall.
#
# HISTORY: this was briefly 0.55 (footprint inscribed radius + pose-error
# margin) to compensate for the BaseObstacle DWB critic, which scored only the
# cell under the robot's centre and so let the planner route the body outside
# the arena. That compensation is no longer needed and it cost too much
# manoeuvring room (drivable width shrank to 3.3 m for a 1.24 m rover):
#   - DWB now uses the ObstacleFootprint critic (nav_params.yaml), which
#     collision-checks the full footprint polygon against the local costmap,
#     where the physical walls are seen by the depth cameras.
#   - The keepout filter applies to the GLOBAL costmap only, so it shapes the
#     planner's corridor; body-level wall safety comes from the sensed walls.
#
# Back to the true arena outline. If the planner is ever seen hugging a wall
# the cameras haven't observed yet, raise this a little (e.g. 0.10-0.20 for
# pose error) rather than returning to 0.55.
CLEARANCE_M = 0.0

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
# round(), not int(): 9.14 / 0.05 = 182.8 px, and truncation would shave 4 cm
# off the far wall.
start_x_px = int(PADDING_M / RESOLUTION)
end_x_px = start_x_px + int(round(ARENA_WIDTH_X / RESOLUTION))

start_y_px = int(PADDING_M / RESOLUTION)
end_y_px = start_y_px + int(round(ARENA_LENGTH_Y / RESOLUTION))

# 3. Paint the inside of the arena as FREE SPACE (Solid White = 255), inset by
#    CLEARANCE_M so the drivable region describes where the rover's CENTRE may
#    go, not where its body may go. Only the painted pixels change here - the
#    image dimensions and the YAML origin below are untouched, so map-frame
#    alignment is unaffected.
clearance_px = int(round(CLEARANCE_M / RESOLUTION))

mask[start_y_px + clearance_px:end_y_px - clearance_px,
     start_x_px + clearance_px:end_x_px - clearance_px] = 255

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

drivable_x = (ARENA_WIDTH_X - 2 * CLEARANCE_M)
drivable_y = (ARENA_LENGTH_Y - 2 * CLEARANCE_M)

print(f"Generated keep-out mask size: {PIX_X}x{PIX_Y} pixels")
print(f"Calculated YAML origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]")
print(f"Arena: {ARENA_WIDTH_X} x {ARENA_LENGTH_Y} m")
print(f"Centre clearance: {CLEARANCE_M} m ({clearance_px} px)")
print(f"Drivable centre region: {drivable_x:.2f} x {drivable_y:.2f} m")