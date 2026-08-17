"""
Generates the arena surface textures used by arena_ucf.world.

Why this exists
---------------
The arena's ground and walls were flat-shaded solid colours. That is fine for
AprilTag work and for the Nav2 voxel layer, but it makes RGB-D visual odometry
(vslam_launch.py) impossible to test: a feature detector needs local intensity
structure, and a constant-colour polygon has none. Measured on the old world,
the front camera saw a mean absolute gradient of 0.39 grey levels per pixel and
rgbd_odometry could only ever match 2-7 inliers against a minimum of 8.

Real BP-1 / LHS-2E regolith is the opposite: fine grained, tonally varied and
covered in the rover's own track marks. Untextured sim was therefore giving a
pessimistic and unrepresentative answer about whether VO is viable.

These textures are deliberately fractal (energy at many scales) so features
survive minification when the camera is looking across the arena, rather than
being pure fine grain that mipmaps average into flat grey at 3 m.

Run
---
    python3 regolith_texture_gen.py

Writes into regolith_model/materials/textures/. Re-run only if you want to
change the look; the PNGs are committed so a fresh clone doesn't need numpy.
"""

import os

import numpy as np
import imageio.v2 as imageio

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "regolith_model", "materials", "textures")

SIZE = 1024
RNG = np.random.default_rng(20260817)


def fractal_noise(size, octaves=7, persistence=0.55):
    """Sum of upsampled random grids — cheap value noise, no dependencies."""
    total = np.zeros((size, size), dtype=np.float32)
    amplitude = 1.0
    norm = 0.0

    for octave in range(octaves):
        res = max(2, 2 ** (octave + 1))
        grid = RNG.random((res, res)).astype(np.float32)

        # Bilinear upsample to full size by index interpolation.
        ys = np.linspace(0, res - 1, size, dtype=np.float32)
        xs = np.linspace(0, res - 1, size, dtype=np.float32)
        y0 = np.floor(ys).astype(int)
        x0 = np.floor(xs).astype(int)
        y1 = np.minimum(y0 + 1, res - 1)
        x1 = np.minimum(x0 + 1, res - 1)
        fy = (ys - y0)[:, None]
        fx = (xs - x0)[None, :]

        top = grid[np.ix_(y0, x0)] * (1 - fx) + grid[np.ix_(y0, x1)] * fx
        bot = grid[np.ix_(y1, x0)] * (1 - fx) + grid[np.ix_(y1, x1)] * fx
        total += amplitude * (top * (1 - fy) + bot * fy)

        norm += amplitude
        amplitude *= persistence

    return total / norm


def add_pebbles(field, count, radius_px, darkness):
    """Scatter soft dark blobs — the dm-scale structure VO actually locks onto."""
    yy, xx = np.mgrid[0:SIZE, 0:SIZE]
    for _ in range(count):
        cy, cx = RNG.integers(0, SIZE, 2)
        r = RNG.uniform(*radius_px)
        # Wrap-aware distance so the texture stays seamless when tiled.
        dy = np.minimum(np.abs(yy - cy), SIZE - np.abs(yy - cy))
        dx = np.minimum(np.abs(xx - cx), SIZE - np.abs(xx - cx))
        d2 = (dy * dy + dx * dx) / (r * r)
        field -= darkness * np.exp(-d2)
    return field


def colourise(grey, base_rgb, contrast):
    """Tint a 0..1 grey field toward an RGB base colour."""
    g = np.clip(0.5 + (grey - grey.mean()) * contrast, 0.0, 1.0)
    rgb = np.stack([g * base_rgb[0], g * base_rgb[1], g * base_rgb[2]], axis=-1)
    return (np.clip(rgb, 0, 1) * 255).astype(np.uint8)


def make_regolith():
    """Ground: fine grain + scattered pebbles, matching the old flat colour."""
    field = fractal_noise(SIZE, octaves=8, persistence=0.58)
    field = add_pebbles(field, count=260, radius_px=(6, 26), darkness=0.16)
    field = add_pebbles(field, count=40, radius_px=(28, 70), darkness=0.10)

    # Fine top-end grain the old flat surface completely lacked.
    field += RNG.normal(0.0, 0.035, (SIZE, SIZE)).astype(np.float32)

    # Base colour is the arena_ground diffuse it replaces (0.45 0.42 0.38),
    # lifted slightly because the texture multiplies it down on average.
    return colourise(field, base_rgb=(0.62, 0.58, 0.52), contrast=1.9)


def make_wall():
    """Walls: anisotropic grain, closer to sheet material than to soil."""
    field = fractal_noise(SIZE, octaves=6, persistence=0.5)

    # Stretch horizontally so the structure reads as a panel surface rather
    # than more dirt — and gives VO gradients in a different direction.
    stretched = np.repeat(field[:, ::8], 8, axis=1)[:, :SIZE]
    field = 0.65 * field + 0.35 * stretched
    field += RNG.normal(0.0, 0.02, (SIZE, SIZE)).astype(np.float32)

    return colourise(field, base_rgb=(0.42, 0.42, 0.43), contrast=1.35)


def report(name, img):
    g = img.mean(axis=2)
    grad = np.abs(np.diff(g, axis=0)[:, :-1]) + np.abs(np.diff(g, axis=1)[:-1, :])
    print(f"{name}: mean={g.mean():6.2f} std={g.std():6.2f} "
          f"gradient={grad.mean():6.3f} grey-levels/px")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    for name, img in [("regolith.png", make_regolith()), ("arena_wall.png", make_wall())]:
        path = os.path.join(OUT_DIR, name)
        imageio.imwrite(path, img)
        report(name, img)
        print(f"  -> {path}")


if __name__ == "__main__":
    main()
