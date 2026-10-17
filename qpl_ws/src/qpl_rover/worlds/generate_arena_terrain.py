"""
Generate the US arena terrain (worlds/arena_terrain/) for Gazebo Classic.

Writes, into arena_terrain/meshes/:
  terrain.obj        one triangle mesh, used for BOTH the visual and the
                     collision in model.sdf, so what you see is exactly what
                     the wheels drive on. Vertices are in arena coordinates
                     (origin at the south-west corner), so the model pose is 0.
  terrain.mtl        matte material pointing at the texture
  sand_diffuse.jpg   one non-tiling texture spanning the whole floor

Run from anywhere:  python3 worlds/generate_arena_terrain.py

Deterministic: same SEED -> same terrain. Needs numpy, scipy, opencv (all
already on the dev machines). The source photo texture is ambientCG
Ground079S (CC0), see arena_terrain/materials/source/README.md.

Why a mesh and not a Gazebo <heightmap>: Gazebo Classic builds separate
physics and render copies of a heightmap and places them differently (render
ignores the model pose), and only reads 8-bit images, so the visible floor and
the floor the rover drives on drifted apart.
"""

import os

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter, map_coordinates

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "arena_terrain")
SOURCE_TEXTURE = os.path.join(OUT_DIR, "materials", "source", "Ground079S_2K_Color.jpg")

SEED = 8                       # picked from a sweep for the most even spread of slopes

# -----------------------------
# Geometry
# -----------------------------
ARENA_X = 9.14                 # US arena interior, origin at the south-west corner
ARENA_Y = 8.10
CENTRE = (ARENA_X / 2, ARENA_Y / 2)
TERRAIN_SIZE = 9.6             # square working grid (and texture) centred on the arena (m)
RES = 513                      # working grid samples per side (~1.9 cm)
WALL_T = 0.1                   # the mesh runs under the walls to their outer faces
MESH_STEP = 0.05               # mesh vertex spacing (m); facet error < 1 mm on these hills

# -----------------------------
# Undulation tuning
# -----------------------------
MAX_SLOPE_DEG = 6.0            # hard cap anywhere on the surface
MAX_AMPLITUDE = 0.10           # hard cap on |height| (m)
MIN_CURVATURE_RADIUS = 0.6     # bumps must be much rounder than the 0.15 m wheels (m)
# (wavelength-ish sigma in metres, relative weight)
OCTAVES = [(1.00, 1.0), (0.50, 0.3)]

# Areas forced flat (z = 0), as (x_min, y_min, x_max, y_max) in arena coords.
BORDER = 0.4                   # flat band along every wall
FALLOFF = 1.0                  # smooth blend distance into the hills (m)
FLAT_ZONES = [
    (2.32, 0.00, 4.57, 2.25),  # starting zone, rover spawns at (3.45, 1.13)
    (7.14, 4.69, 9.14, 8.10),  # construction / berm zone
]

# -----------------------------
# Future features (empty for now)
# -----------------------------
# Craters are carved into the terrain mesh: (x, y, diameter) in arena metres.
# Boulders would be separate static models placed on the surface in the world
# file, so they can have their own shape and material.
FEATURES = {
    "craters": [],             # e.g. (6.0, 3.0, 0.5)
    "boulders": [],            # e.g. (5.0, 5.0, 0.3)  - not implemented yet
}

# -----------------------------
# Texture tuning
# -----------------------------
TEX_RES = 2048                 # ~4.7 mm per pixel over 9.6 m, ~7 MB
TEX_SOURCE_TILE_M = 1.0        # metres covered by one copy of the source photo
TEX_SATURATION = 0.45          # pull the yellow beach sand towards grey regolith
PEBBLES_PER_M2 = 40

PX = TERRAIN_SIZE / (RES - 1)  # metres per working grid cell


def arena_grid(res):
    """World (x, y) of every working grid sample. Row 0 is +y (north), column 0 is -x (west)."""
    half = TERRAIN_SIZE / 2
    xs = np.linspace(CENTRE[0] - half, CENTRE[0] + half, res)
    ys = np.linspace(CENTRE[1] + half, CENTRE[1] - half, res)
    return np.meshgrid(xs, ys)


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


def rect_distance(x, y, rect):
    """Distance from each point to an axis-aligned rectangle (0 inside)."""
    x0, y0, x1, y1 = rect
    dx = np.maximum(np.maximum(x0 - x, 0), x - x1)
    dy = np.maximum(np.maximum(y0 - y, 0), y - y1)
    return np.hypot(dx, dy)


def hill_weight(x, y):
    """1 where hills are allowed, 0 where the floor must be flat, smooth between."""
    # distance inward from the walls (negative outside the arena)
    inward = np.minimum.reduce([x, ARENA_X - x, y, ARENA_Y - y])
    weight = smoothstep((inward - BORDER) / FALLOFF)
    for rect in FLAT_ZONES:
        weight *= smoothstep(rect_distance(x, y, rect) / FALLOFF)
    return weight


def add_crater(h, x, y, cx, cy, diameter):
    """Bowl with a raised rim. Same proportions as draw_crater_sharp() in
    terrain_heightmap_gen.py (total depth 0.22 x diameter, 60 % of it rim),
    but built from cosine segments so the surface has no kinks or lip edges."""
    r = diameter / 2
    depth = 0.22 * diameter
    rim = 0.6 * depth
    floor = depth - rim
    rim_width = 0.45                     # outer rim slope, in crater radii
    d = np.hypot(x - cx, y - cy) / r
    inner = -floor + depth * 0.5 * (1 - np.cos(np.pi * np.clip(d, 0, 1)))
    outer = rim * 0.5 * (1 + np.cos(np.pi * np.clip((d - 1) / rim_width, 0, 1)))
    return h + np.where(d < 1, inner, outer)


def slope_stats(h):
    gy, gx = np.gradient(h, PX)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    gxx = np.gradient(gx, PX, axis=1)
    gyy = np.gradient(gy, PX, axis=0)
    curvature = np.max(np.abs(gxx) + np.abs(gyy))
    return slope, curvature


def make_heightmap():
    rng = np.random.default_rng(SEED)
    x, y = arena_grid(RES)

    h = np.zeros((RES, RES))
    for sigma_m, weight in OCTAVES:
        layer = gaussian_filter(rng.standard_normal((RES, RES)), sigma_m / PX, mode="reflect")
        h += weight * layer / layer.std()
    h -= h.mean()
    h *= hill_weight(x, y)

    # Scale so all three limits hold; slope and curvature are linear in scale.
    slope, curvature = slope_stats(h)
    scale = min(
        MAX_AMPLITUDE / np.abs(h).max(),
        np.tan(np.radians(MAX_SLOPE_DEG)) / np.tan(np.radians(slope.max())),
        (1 / MIN_CURVATURE_RADIUS) / curvature,
    )
    h *= scale

    for cx, cy, diameter in FEATURES["craters"]:
        h = add_crater(h, x, y, cx, cy, diameter)

    slope, curvature = slope_stats(h)
    flat = hill_weight(x, y) == 0
    print(f"height      min {h.min() * 100:+.1f} cm  max {h.max() * 100:+.1f} cm")
    print(f"slope       max {slope.max():.1f} deg  mean {slope[~flat].mean():.1f} deg")
    print(f"curvature   min radius {1 / curvature:.2f} m")
    print(f"flat zones  max |z| {np.abs(h[flat]).max() * 1000:.1f} mm")
    if FEATURES["craters"]:
        print("note: craters are not slope-limited, check the numbers above")
    else:
        assert slope.max() <= MAX_SLOPE_DEG + 1e-6
    return h


def rotated_layer(src, angle_deg, scale, offset, size):
    """Tile a seamless source texture across `size`, rotated and shifted, with no seams."""
    centre = (size / 2, size / 2)
    m = cv2.getRotationMatrix2D(centre, angle_deg, scale)
    m[:, 2] += offset
    return cv2.warpAffine(src, cv2.invertAffineTransform(m), (size, size),
                          flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_WRAP)


def make_texture():
    rng = np.random.default_rng(SEED + 1)
    src = cv2.imread(SOURCE_TEXTURE, cv2.IMREAD_COLOR).astype(np.float32)
    tile_px = TEX_RES * TEX_SOURCE_TILE_M / TERRAIN_SIZE
    src = cv2.resize(src, (int(round(tile_px)),) * 2, interpolation=cv2.INTER_AREA)

    # Three copies of the photo at different angles/offsets, blended by smooth
    # noise masks, so no two patches of floor look the same to the VO.
    layers = [rotated_layer(src, rng.uniform(0, 360), rng.uniform(0.85, 1.15),
                            rng.uniform(0, tile_px, 2), TEX_RES) for _ in range(3)]
    low = TEX_RES // 64
    masks = [cv2.resize(gaussian_filter(rng.random((low, low)), 2), (TEX_RES, TEX_RES),
                        interpolation=cv2.INTER_CUBIC) for _ in layers]
    masks = np.stack(masks) ** 4
    masks /= masks.sum(axis=0)
    tex = sum(layer * m[..., None] for layer, m in zip(layers, masks))

    # Grey it towards regolith and add large-scale albedo patches.
    grey = tex.mean(axis=2, keepdims=True)
    tex = grey + TEX_SATURATION * (tex - grey)
    albedo = gaussian_filter(rng.standard_normal((low, low)), 3)
    albedo = cv2.resize(albedo / albedo.std(), (TEX_RES, TEX_RES), interpolation=cv2.INTER_CUBIC)
    tex *= (1 + 0.08 * albedo)[..., None]

    # Mid-scale mottling (scuffs, clumps, ~5-50 cm). The cameras look along the
    # floor at a grazing angle and Gazebo samples blurry mip levels there, so
    # fine grain turns to flat grey past ~1.5 m; this scale is what survives.
    px_per_m = TEX_RES / TERRAIN_SIZE
    for sigma_m, amount in [(0.015, 0.06), (0.06, 0.14), (0.20, 0.10)]:
        mottle = gaussian_filter(rng.standard_normal((TEX_RES, TEX_RES)).astype(np.float32),
                                 sigma_m * px_per_m)
        tex *= (1 + amount * mottle / mottle.std())[..., None]

    # Scattered pebbles and clods: strong corners for the feature detector.
    # Mostly small and mostly darker than the sand, like real gravel in regolith.
    n = int(PEBBLES_PER_M2 * TERRAIN_SIZE ** 2)
    for _ in range(n):
        cx, cy = rng.uniform(0, TEX_RES, 2)
        a = min(0.006 + rng.exponential(0.012), 0.08) * px_per_m
        b = a * rng.uniform(0.5, 1.0)
        shade = rng.choice([0.45, 0.55, 0.65, 0.75, 1.15, 1.25])
        x0, y0 = int(max(cx - a - 2, 0)), int(max(cy - a - 2, 0))
        x1, y1 = int(min(cx + a + 3, TEX_RES)), int(min(cy + a + 3, TEX_RES))
        mask = np.zeros((y1 - y0, x1 - x0), np.float32)
        cv2.ellipse(mask, (int(cx - x0), int(cy - y0)), (max(int(a), 1), max(int(b), 1)),
                    rng.uniform(0, 180), 0, 360, 1.0, -1, cv2.LINE_AA)
        patch = tex[y0:y1, x0:x1]
        patch *= (1 + (shade - 1) * mask)[..., None]

    return np.clip(tex, 0, 255).astype(np.uint8)


def write_mesh(h, mesh_dir):
    """Resample the working grid onto a MESH_STEP grid covering the arena out
    to the walls' outer faces and write it as an OBJ with normals and UVs."""
    x0, x1 = -WALL_T, ARENA_X + WALL_T
    y0, y1 = -WALL_T, ARENA_Y + WALL_T
    nx = int(round((x1 - x0) / MESH_STEP)) + 1
    ny = int(round((y1 - y0) / MESH_STEP)) + 1
    xs, ys = np.linspace(x0, x1, nx), np.linspace(y0, y1, ny)
    X, Y = np.meshgrid(xs, ys)                     # row i = y, column j = x

    half = TERRAIN_SIZE / 2
    col = (X - (CENTRE[0] - half)) / PX
    row = ((CENTRE[1] + half) - Y) / PX
    Z = map_coordinates(h, [row, col], order=3, mode="nearest")

    gy, gx = np.gradient(Z, ys, xs)
    normals = np.stack([-gx, -gy, np.ones_like(Z)], axis=-1)
    normals /= np.linalg.norm(normals, axis=-1, keepdims=True)

    # UVs place the square texture over the same square as the working grid;
    # Gazebo's OBJ loader flips v, so v = 1 is the top (north) row of the image.
    u = (X - (CENTRE[0] - half)) / TERRAIN_SIZE
    v = (Y - (CENTRE[1] - half)) / TERRAIN_SIZE

    idx = np.arange(nx * ny).reshape(ny, nx) + 1   # OBJ is 1-based
    a, b = idx[:-1, :-1], idx[:-1, 1:]
    c, d = idx[1:, 1:], idx[1:, :-1]
    # counter-clockwise seen from above, so normals point up
    tris = np.concatenate([np.stack([a, b, c], -1).reshape(-1, 3),
                           np.stack([a, c, d], -1).reshape(-1, 3)])

    path = os.path.join(mesh_dir, "terrain.obj")
    with open(path, "w") as f:
        f.write("# Generated by worlds/generate_arena_terrain.py - do not edit\n")
        f.write("mtllib terrain.mtl\no arena_terrain\n")
        np.savetxt(f, np.stack([X, Y, Z], -1).reshape(-1, 3), fmt="v %.4f %.4f %.5f")
        np.savetxt(f, np.stack([u, v], -1).reshape(-1, 2), fmt="vt %.5f %.5f")
        np.savetxt(f, normals.reshape(-1, 3), fmt="vn %.4f %.4f %.4f")
        f.write("usemtl sand\n")
        np.savetxt(f, np.repeat(tris, 3, axis=1), fmt="f %d/%d/%d %d/%d/%d %d/%d/%d")

    with open(os.path.join(mesh_dir, "terrain.mtl"), "w") as f:
        f.write("# Generated by worlds/generate_arena_terrain.py - do not edit\n"
                "newmtl sand\n"
                "Ka 1.0 1.0 1.0\n"
                "Kd 1.0 1.0 1.0\n"
                "Ks 0.0 0.0 0.0\n"          # matte: shine washes the floor out on camera
                "illum 1\n"
                "map_Kd sand_diffuse.jpg\n")

    print(f"mesh        {nx} x {ny} vertices, {len(tris)} triangles, "
          f"{os.path.getsize(path) / 1e6:.1f} MB")


def main():
    mesh_dir = os.path.join(OUT_DIR, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)

    h = make_heightmap()
    write_mesh(h, mesh_dir)

    tex_path = os.path.join(mesh_dir, "sand_diffuse.jpg")
    cv2.imwrite(tex_path, make_texture(), [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(f"texture     {os.path.getsize(tex_path) / 1e6:.1f} MB")
    print(f"wrote {mesh_dir}/")


if __name__ == "__main__":
    main()
