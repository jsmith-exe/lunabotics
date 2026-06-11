import numpy as np
import imageio.v2 as imageio
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# -----------------------------
# STRICT 1-METER GAZEBO CALIBRATION
# -----------------------------
SIZE = 257
TOTAL_Z_RANGE = 1.0  # Kept at 1.0m to maintain working scale tag

# Baseline floor sits exactly at perfect mid-gray (128)
BASE_HEIGHT = 128
terrain = np.ones((SIZE, SIZE), dtype=np.float32) * BASE_HEIGHT

# Precise conversion: exactly 255 grayscale steps per 1.0 meter of height
METRIC_TO_GRAYSCALE = 255.0 / TOTAL_Z_RANGE

# Track active crater footprints to protect them from dune deformation
crater_mask_accumulated = np.zeros((SIZE, SIZE), dtype=bool)
crater_inner_mask_accumulated = np.zeros((SIZE, SIZE), dtype=bool)


def meter_to_pixel(val):
    return int((val / 7.9) * 257)


# -----------------------------
# SINGLE-INPUT CRATER DRAWER (STRICT DIAMS)
# -----------------------------
def draw_crater_sharp(cx_m, cy_m, diameter_m):
    global crater_mask_accumulated, crater_inner_mask_accumulated

    total_depth_m = diameter_m * 0.22  # 0.5m diameter = 11cm total depth (top of rim to bottom)
    rim_height_m = total_depth_m * 0.60  # 6.6cm sits above base ground
    sub_floor_depth_m = total_depth_m - rim_height_m  # 4.4cm drops below base ground

    cx_px = meter_to_pixel(cx_m + 1.75)
    cy_px = meter_to_pixel(cy_m)
    cy_px = (SIZE - 1) - cy_px

    r_px = meter_to_pixel(diameter_m / 2.0)
    rim_width_px = int(r_px * 0.45)
    total_radius = r_px + rim_width_px

    y, x = np.ogrid[:SIZE, :SIZE]
    dist = np.sqrt((x - cx_px) ** 2 + (y - cy_px) ** 2)
    active_mask = dist <= total_radius

    if np.any(active_mask):
        crater_mask_accumulated |= active_mask

        d = dist[active_mask]
        profile = np.zeros_like(d, dtype=np.float32)

        # ZONE A: Inner Bowl (Strictly within the requested diameter)
        inner_mask = d < r_px
        if np.any(inner_mask):
            crater_inner_mask_accumulated |= (dist < r_px)
            bowl_profile = sub_floor_depth_m * (1 - (d[inner_mask] / r_px) ** 2)
            profile[inner_mask] = -bowl_profile

        # ZONE B: Sharp Triangular Rim (Pushed completely above the baseline)
        rim_inner_mask = (d >= (r_px - rim_width_px)) & (d < r_px)
        if np.any(rim_inner_mask):
            t_in = (d[rim_inner_mask] - (r_px - rim_width_px)) / rim_width_px
            profile[rim_inner_mask] += rim_height_m * t_in

        rim_outer_mask = (d >= r_px) & (d <= total_radius)
        if np.any(rim_outer_mask):
            t_out = (total_radius - d[rim_outer_mask]) / rim_width_px
            profile[rim_outer_mask] = rim_height_m * t_out

        gray_profile = profile * METRIC_TO_GRAYSCALE
        terrain[active_mask] += gray_profile


# -----------------------------
# GENERATE RIGID MECHANICAL DUNES
# -----------------------------
"""
y_indices, x_indices = np.indices((SIZE, SIZE), dtype=np.float32)

# Wave System 1: Broad rolling dunes slanting across the field (15cm high peaks)
dune_wave_1 = 0.08 * np.sin((x_indices * 0.04) + (y_indices * 0.03))

# Wave System 2: Sharp, high-frequency shovel ridges run perpendicular to provide micro-slopes
dune_wave_2 = 0.04 * np.sin((x_indices * 0.18) - (y_indices * 0.12))

# Combine wave fields into a physical sand displacement field
dune_field_meters = dune_wave_1 + dune_wave_2

# Define the Flat Control Oasis (Top right corner of the field: x > 5.0m, y < 2.5m)
flat_zone_mask = (x_indices > meter_to_pixel(5.0)) & (y_indices < meter_to_pixel(2.5))

# Environmental modifier grid: Blends out the dunes near craters and flat zone
dune_modifier = np.ones((SIZE, SIZE), dtype=np.float32)
dune_modifier[flat_zone_mask] = 0.00  # Zone 3: Perfectly flat oasis
dune_modifier[crater_inner_mask_accumulated] = 0.00  # Keep inside of craters driveable
dune_modifier[crater_mask_accumulated] = 0.20  # Scale down noise around the rim

# Layer the actual physical dunes onto our baseline ground mesh
terrain += (dune_field_meters * dune_modifier) * METRIC_TO_GRAYSCALE
"""

# -----------------------------
# PLACEMENT OF MULTI-SCALE HAZARDS
# -----------------------------
"""
# Test Case 1: Strict Rulebook Compliant Craters (Zone 1)
draw_crater_sharp(cx_m=0.8, cy_m=5.5, diameter_m=0.50)
draw_crater_sharp(cx_m=2.2, cy_m=6.5, diameter_m=0.40)

# Test Case 2: Massive Photo-Realistic Anomaly Crater (Zone 2)
draw_crater_sharp(cx_m=1.2, cy_m=2.5, diameter_m=1.00)
"""

# -----------------------------
# EXPORT ASSET
# -----------------------------
terrain = np.clip(terrain, 0, 255).astype(np.uint8)

out_path = os.path.join(BASE_DIR, "terrain_heightmap", "heightmap.png")
os.makedirs(os.path.dirname(out_path), exist_ok=True)
imageio.imwrite(out_path, terrain)

print("Saved highly structured, sweeping sand dune heightmap to:", out_path)