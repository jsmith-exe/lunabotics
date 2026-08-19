"""
Generates the arena surface textures used by the sim worlds.

Why this exists
---------------
The arena's ground and walls were flat-shaded solid colours. That is fine for
AprilTag work and for the Nav2 voxel layer, but it makes RGB-D visual odometry
(vslam_launch.py) impossible to test: a feature detector needs local intensity
structure, and a constant-colour polygon has none. Measured on the untextured
world, the front camera saw a mean absolute gradient of 0.39 grey levels per
pixel and rgbd_odometry could only ever match 2-7 inliers against a minimum
of 8.

Real BP-1 / LHS-2E regolith is the opposite: fine grained, tonally varied,
pocked with craters and scattered rock, and covered in the rover's own track
marks. Untextured sim was therefore giving a pessimistic and unrepresentative
answer about whether VO is viable.

What to optimise for
--------------------
NOT pixel-level grain. The camera undersamples the texture, so fine grain is
mipmapped into flat grey before it ever reaches the feature detector:

    depth camera   848 x 480 over 1.51844 rad HFOV  ->  1.79 mrad/px
    at 3 m range   ->  5.4 mm per camera pixel
    ground texture 2 m tile / 2048 px               ->  1.0 mm per texel

So the camera averages ~5x5 texels per pixel at 3 m and worse further out.
What survives that averaging is DECIMETRE-TO-METRE structure, which is why
this generator is built around craters, shadowed rocks and broad albedo
patches rather than noise alone. Each feature class below is sized in metres
first and converted to pixels via PX_PER_M.

Shading is baked into the diffuse map. That is a hack - Gazebo lights the
surface itself - but a lit sphere and a flat disc of the same albedo are
indistinguishable to a feature detector, and baked relief is what produces
gradients that survive minification. Light comes from +X to match the world's
directional sun (direction -0.5 0.1 -0.9).

Everything is wrap-aware so the textures tile seamlessly.

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

# Per-texture resolution, chosen from viewing distance rather than uniformly.
#
#   Ground  2 m tile at 2048 px  -> 0.98 mm/texel. The ground runs right up to
#           the rover's wheels, and at 0.5 m the camera resolves 0.9 mm/px, so
#           this is matched rather than wasteful.
#   Wall    0.4 m tile at 1024 px -> 0.39 mm/texel. Walls are never seen closer
#           than ~1 m and usually much further, where the camera resolves
#           1.8 mm/px or worse. 2048 here would be four times the bytes for
#           detail no camera in this sim can resolve.
#
# Both are set into SIZE before the corresponding make_* call. Halve them if
# the committed PNGs are too heavy for the repo; the structure survives, only
# the finest grain is lost.
GROUND_SIZE = 2048
WALL_SIZE = 1024

GROUND_TILE_M = 2.0                 # must match the world's texture <size>
WALL_TILE_M = 0.40                  # must match regolith.material's wall scale

SIZE = GROUND_SIZE                  # set per texture in main()
PX_PER_M = SIZE / GROUND_TILE_M
RNG = np.random.default_rng(20260818)

# Unit light direction in texture space (from +X, slightly +Y), matching the
# world's <direction>-0.5 0.1 -0.9</direction> sun.
LIGHT = np.array([0.98, 0.20]) / np.linalg.norm([0.98, 0.20])


# --------------------------------------------------------------------------
# Base noise
# --------------------------------------------------------------------------
def fractal_noise(size, octaves=8, persistence=0.55, rng=RNG):
    """Sum of upsampled random grids - cheap value noise, no dependencies."""
    total = np.zeros((size, size), dtype=np.float32)
    amplitude = 1.0
    norm = 0.0

    for octave in range(octaves):
        res = max(2, 2 ** (octave + 1))
        grid = rng.random((res, res)).astype(np.float32)

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


def _patch(field, cy, cx, r):
    """Local wrap-around view helper.

    Returns (rows, cols, dy, dx) index arrays for a square patch of radius r
    centred on (cy, cx), wrapped modulo SIZE so features straddling an edge
    reappear on the opposite side and the tile stays seamless. Working on a
    patch instead of the whole array keeps this fast enough to scatter
    thousands of features at 2048x2048.
    """
    r = int(np.ceil(r))
    off = np.arange(-r, r + 1)
    rows = (cy + off) % SIZE
    cols = (cx + off) % SIZE
    dy = off[:, None].astype(np.float32)
    dx = off[None, :].astype(np.float32)
    return rows, cols, dy, dx


# --------------------------------------------------------------------------
# Ground features
# --------------------------------------------------------------------------
def add_craters(field, count, diam_m, depth):
    """Bowl-shaped depressions with a lit rim and a shadowed interior.

    The single most recognisable lunar feature, and the most useful one here:
    a crater puts a bright arc and a dark arc a few centimetres apart, which
    is exactly the kind of high-contrast, rotation-stable structure a feature
    detector locks onto and can still resolve from across the arena.
    """
    for _ in range(count):
        d = RNG.uniform(*diam_m) * PX_PER_M
        r = d / 2.0
        cy, cx = RNG.integers(0, SIZE, 2)
        rows, cols, dy, dx = _patch(field, cy, cx, r * 1.35)

        dist = np.sqrt(dy * dy + dx * dx) / r          # 1.0 at the rim
        inside = dist < 1.0
        rim = (dist >= 1.0) & (dist < 1.35)

        # Interior: parabolic bowl. Surface normal tilts with the local slope,
        # so the wall facing the light brightens and the far wall darkens.
        slope = np.where(inside, dist, 0.0)
        nx = np.where(dist > 1e-6, dx / np.maximum(dist * r, 1e-6), 0.0)
        ny = np.where(dist > 1e-6, dy / np.maximum(dist * r, 1e-6), 0.0)
        lit = (nx * LIGHT[0] + ny * LIGHT[1]) * slope

        bowl = np.where(inside, -depth * (1.0 - dist ** 2) + depth * 0.9 * lit, 0.0)

        # Raised ejecta rim, brightest where it faces the light.
        rim_lit = 0.18 * depth * (1.0 + (nx * LIGHT[0] + ny * LIGHT[1]))
        bowl = bowl + np.where(rim, rim_lit, 0.0)

        field[np.ix_(rows, cols)] += bowl
    return field


def add_rocks(field, count, diam_m, height):
    """Scattered rock: irregular, varied albedo, with a cast shadow.

    Three things matter here and all three were wrong in the first pass:

      * Outline. Perfect circles read as pasted discs. Each rock's radius is
        modulated by a few angular harmonics so the silhouette is irregular,
        which is also what makes the gradient direction vary around the edge
        rather than being purely radial.
      * Albedo. Real scatter includes rock both lighter and darker than the
        soil. Uniformly bright caps made the surface look like foam.
      * Edge softness. A hard cutoff produces a ring artefact under
        minification; a smooth falloff degrades gracefully instead.
    """
    for _ in range(count):
        d = RNG.uniform(*diam_m) * PX_PER_M
        r = max(d / 2.0, 1.5)
        cy, cx = RNG.integers(0, SIZE, 2)

        # Irregular outline: 2nd/3rd/5th angular harmonics, random phase.
        h = RNG.uniform(-1, 1, 3) * (0.10, 0.16, 0.09)
        ph = RNG.uniform(0, 2 * np.pi, 3)

        # Albedo: mostly lighter than soil, but a third of it darker.
        albedo = RNG.uniform(0.45, 1.5) * (-0.55 if RNG.random() < 0.33 else 1.0)

        # Shadow first, so the rock draws over it. Skip it for dark rock,
        # which is mostly half-buried and casts nothing worth modelling.
        if albedo > 0:
            sh_off = r * 1.15
            sy = int(cy - LIGHT[1] * sh_off)
            sx = int(cx - LIGHT[0] * sh_off)
            rows, cols, dy, dx = _patch(field, sy, sx, r * 1.9)
            d2 = (dy * dy + dx * dx) / (r * 0.95) ** 2
            field[np.ix_(rows, cols)] -= height * 0.45 * np.exp(-d2)

        rows, cols, dy, dx = _patch(field, cy, cx, r * 1.6)
        ang = np.arctan2(dy, dx)
        wobble = 1.0 + sum(h[k] * np.sin((k + 2) * ang + ph[k]) for k in range(3))
        dist = np.sqrt(dy * dy + dx * dx) / np.maximum(r * wobble, 1e-6)

        # Smooth falloff to zero over the outer 25% of the radius.
        mask = np.clip((1.0 - dist) / 0.14, 0.0, 1.0)
        mask = mask * mask * (3 - 2 * mask)          # smoothstep

        nz = np.sqrt(np.clip(1.0 - np.minimum(dist, 1.0) ** 2, 0.0, 1.0))
        rr = np.maximum(np.sqrt(dy * dy + dx * dx), 1e-6)
        nx, ny = dx / rr, dy / rr
        shade = np.clip(nx * LIGHT[0] + ny * LIGHT[1] + nz * 0.40, -1, 1)
        field[np.ix_(rows, cols)] += mask * height * albedo * shade
    return field


def add_track_marks(field, count, width_m, darkness):
    """Faint parallel wheel ruts.

    Real arenas are driven over before anyone measures anything, and ruts add
    long straight gradients in a different orientation to the blobby crater
    and rock structure - useful for constraining yaw.
    """
    yy, xx = np.mgrid[0:SIZE, 0:SIZE].astype(np.float32)
    for _ in range(count):
        theta = RNG.uniform(0, np.pi)
        nx, ny = np.cos(theta), np.sin(theta)
        offset = RNG.uniform(0, SIZE)
        w = width_m * PX_PER_M
        # Wrapped perpendicular distance to an infinite line.
        proj = (xx * nx + yy * ny + offset) % SIZE
        dist = np.minimum(proj, SIZE - proj)
        rut = np.exp(-(dist / w) ** 2)
        # Two ruts a wheelbase apart, with lightly disturbed soil between.
        proj2 = (proj + 0.745 * PX_PER_M) % SIZE
        dist2 = np.minimum(proj2, SIZE - proj2)
        rut = rut + np.exp(-(dist2 / w) ** 2)
        field -= darkness * rut * RNG.uniform(0.6, 1.0)
    return field


# --------------------------------------------------------------------------
# Colourisation
# --------------------------------------------------------------------------
def colourise(grey, base_rgb, lo=0.30, hi=0.95, tint_strength=0.06):
    """Map a field to RGB via percentile tone mapping.

    Percentiles rather than a fixed contrast multiplier: crater and rock
    relief accumulates additively, so the field's range varies with how many
    features were scattered. A fixed multiplier crushed 25% of the ground to
    pure black, which removes gradient exactly where the relief is densest -
    the opposite of what this texture is for. Mapping p2..p98 into [lo, hi]
    guarantees the full tonal range is used and nothing clips.
    """
    p2, p98 = np.percentile(grey, 2), np.percentile(grey, 98)
    g = (grey - p2) / max(p98 - p2, 1e-6)
    g = np.clip(lo + g * (hi - lo), 0.0, 1.0)

    drift = fractal_noise(SIZE, octaves=4, persistence=0.6)
    drift = (drift - drift.mean()) * tint_strength

    rgb = np.stack([
        g * (base_rgb[0] + drift),
        g * base_rgb[1],
        g * (base_rgb[2] - drift),
    ], axis=-1)
    return (np.clip(rgb, 0, 1) * 255).astype(np.uint8)


# --------------------------------------------------------------------------
# Textures
# --------------------------------------------------------------------------
def make_regolith():
    """Lunar floor: albedo patches, craters, rock, ruts, then fine grain."""
    # Base. Weighted toward the high octaves: a surface photographed from
    # rover height is mostly fine granular soil, and low-frequency blobs read
    # as smudge rather than structure once several overlap.
    field  = fractal_noise(SIZE, octaves=4, persistence=0.62) * 0.22
    field += fractal_noise(SIZE, octaves=9, persistence=0.60) * 0.78

    # Craters, shallow and sparse. A dense crater field is what the Moon looks
    # like from ORBIT; from rover height a 2 m patch of mare has very few, and
    # they are soft dishes rather than pits.
    field = add_craters(field, count=1,  diam_m=(0.40, 0.70), depth=0.030)
    field = add_craters(field, count=3,  diam_m=(0.12, 0.32), depth=0.032)
    field = add_craters(field, count=12, diam_m=(0.03, 0.12), depth=0.038)

    # Rock carries almost all of the trackable structure. Smaller and
    # higher-contrast than the craters: distinct grains, not soft mounds.
    field = add_rocks(field, count=25,   diam_m=(0.06, 0.13),   height=0.115)
    field = add_rocks(field, count=250,  diam_m=(0.02, 0.055),  height=0.105)
    field = add_rocks(field, count=1500, diam_m=(0.004, 0.018), height=0.090)

    field = add_track_marks(field, count=3, width_m=0.05, darkness=0.022)

    # Fine grain last, so it sits on top of the relief rather than under it.
    field += RNG.normal(0.0, 0.038, (SIZE, SIZE)).astype(np.float32)

    # Lunar mare is genuinely dark (albedo ~0.10-0.14) but a near-black diffuse
    # gives the camera nothing to work with once Gazebo's lighting is applied,
    # so this sits at the bright end of plausible.
    return colourise(field, base_rgb=(0.66, 0.62, 0.56), lo=0.30, hi=0.97)


def make_wall():
    """Arena wall: panelled board with fasteners, scuffing and dust.

    Walls matter more than they look. At rover camera height they occupy the
    upper part of the frame and, crucially, they stay in view during turns -
    which is exactly when VO was losing tracking (measured: 15-22% zero-inlier
    frames, clustered in the turning phases of the drive pattern). The ground
    sweeps past during a turn; the far wall does not, so it is the stabler
    reference of the two.

    Built for contrast rather than realism. Structure is coarse and blocky on
    purpose so it survives being viewed from across the arena at a steep
    grazing angle, where a fine grain would mipmap to flat grey.
    """
    # Board grain: strongly anisotropic, so it reads as sheet material and
    # supplies gradient on a different axis from the ground's isotropic soil.
    field = fractal_noise(SIZE, octaves=8, persistence=0.55)
    stretched = np.repeat(field[:, ::12], 12, axis=1)[:, :SIZE]
    field = 0.45 * field + 0.55 * stretched
    field += fractal_noise(SIZE, octaves=6, persistence=0.60) * 0.45

    yy, xx = np.mgrid[0:SIZE, 0:SIZE].astype(np.float32)

    # Four panel joints rather than eight: bigger, plainly visible blocks.
    seam_period = SIZE / 6.0
    ph = (xx % seam_period) / seam_period
    field += -0.55 * np.exp(-((ph - 0.5) / 0.006) ** 2)     # dark groove
    field += 0.34 * np.exp(-((ph - 0.522) / 0.005) ** 2)    # lit lip

    # Per-panel tone: real boards do not match each other.
    panel = (xx // seam_period).astype(int)
    for k in range(6):
        field += np.where(panel == k, RNG.uniform(-0.12, 0.12), 0.0)

    # Horizontal rail across the middle.
    d = np.abs(yy - 0.5 * SIZE)
    field += -0.42 * np.exp(-(d / (SIZE * 0.010)) ** 2)
    field += 0.30 * np.exp(-((yy - (0.5 * SIZE + SIZE * 0.017)) / (SIZE * 0.008)) ** 2)

    # Fasteners: large enough to survive minification, high contrast, and
    # rotation-stable - a good corner feature is worth more than any amount
    # of smooth shading.
    for col in range(6):
        for row in range(8):
            cx = int((col + 0.5) * seam_period + RNG.integers(-10, 11))
            cy = int((row + 0.5) * SIZE / 8.0 + RNG.integers(-10, 11))
            r = RNG.uniform(11, 16)
            rows, cols, dy, dx = _patch(field, cy, cx, r * 2.4)
            dist = np.sqrt(dy * dy + dx * dx) / r
            head = np.clip((1.0 - dist) / 0.30, 0, 1)
            head = head * head * (3 - 2 * head)
            nz = np.sqrt(np.clip(1.0 - np.minimum(dist, 1.0) ** 2, 0, 1))
            rr = np.maximum(np.sqrt(dy * dy + dx * dx), 1e-6)
            shade = (dx / rr) * LIGHT[0] + (dy / rr) * LIGHT[1] + nz * 0.5
            field[np.ix_(rows, cols)] += head * (0.42 * shade - 0.10)
            # Cast shadow under the head.
            sy, sx = int(cy - LIGHT[1] * r * 1.3), int(cx - LIGHT[0] * r * 1.3)
            rows, cols, dy, dx = _patch(field, sy, sx, r * 1.6)
            field[np.ix_(rows, cols)] -= 0.20 * np.exp(-(dy * dy + dx * dx) / (r * 0.9) ** 2)

    # Regolith thrown up the lower third by the drum and the wheels.
    splat = fractal_noise(SIZE, octaves=8, persistence=0.62)
    lower = np.clip((yy / SIZE - 0.60) / 0.40, 0, 1) ** 1.5
    field -= 1.05 * lower * (splat - splat.mean() + 0.42)

    # Scuffs where the rover has clipped the wall, and general scratching.
    # Windowed along their own axis so they are finite streaks: unbounded
    # lines tile into a regular crosshatch, which reads as a net rather than
    # as wear, and gives VO a repeating pattern that is actively unhelpful
    # for disambiguating position along the wall.
    for _ in range(26):
        theta = RNG.uniform(-0.35, 0.35)
        st, ct = np.sin(theta), np.cos(theta)
        offset = RNG.uniform(0, SIZE)
        proj = (xx * st + yy * ct + offset) % SIZE
        dd = np.minimum(proj, SIZE - proj)
        along = xx * ct - yy * st
        centre = RNG.uniform(along.min(), along.max())
        window = np.exp(-((along - centre) / RNG.uniform(120, 380)) ** 2)
        field += RNG.uniform(-0.24, 0.18) * window * \
            np.exp(-(dd / RNG.uniform(1.5, 4.0)) ** 2)

    # Blotchy wear patches. These carry most of the wall's surviving gradient
    # once minified - unlike the scratches they are irregular and non-repeating,
    # so they help VO disambiguate position along the wall instead of hindering it.
    for _ in range(70):
        cy, cx = RNG.integers(0, SIZE, 2)
        r = RNG.uniform(18, 70)
        rows, cols, dy, dx = _patch(field, cy, cx, r * 1.5)
        field[np.ix_(rows, cols)] += RNG.uniform(-0.34, 0.26) * \
            np.exp(-(dy * dy + dx * dx) / (r * r))

    field += RNG.normal(0.0, 0.030, (SIZE, SIZE)).astype(np.float32)

    return colourise(field, base_rgb=(0.56, 0.56, 0.58), lo=0.22, hi=0.98,
                     tint_strength=0.03)


def report(name, img):
    g = img.mean(axis=2)
    grad = np.abs(np.diff(g, axis=0)[:, :-1]) + np.abs(np.diff(g, axis=1)[:-1, :])
    # Gradient after 4x box-downsampling, i.e. roughly what the camera sees at
    # ~4 m once mipmapping has averaged the fine grain away. This is the number
    # that actually predicts whether VO can hold inliers at range.
    s = g[:g.shape[0] // 4 * 4, :g.shape[1] // 4 * 4]
    small = s.reshape(s.shape[0] // 4, 4, s.shape[1] // 4, 4).mean(axis=(1, 3))
    sgrad = np.abs(np.diff(small, axis=0)[:, :-1]) + np.abs(np.diff(small, axis=1)[:-1, :])
    print(f"{name}: {img.shape[1]}x{img.shape[0]} mean={g.mean():6.2f} std={g.std():6.2f}")
    print(f"    gradient  full-res {grad.mean():6.3f}   4x-minified {sgrad.mean():6.3f} grey-levels/px")


def main():
    global SIZE, PX_PER_M
    os.makedirs(OUT_DIR, exist_ok=True)

    SIZE, PX_PER_M = GROUND_SIZE, GROUND_SIZE / GROUND_TILE_M
    ground = make_regolith()
    SIZE, PX_PER_M = WALL_SIZE, WALL_SIZE / WALL_TILE_M
    wall = make_wall()

    for name, img in [("regolith.png", ground), ("arena_wall.png", wall)]:
        path = os.path.join(OUT_DIR, name)
        imageio.imwrite(path, img)
        report(name, img)
        print(f"  -> {path}")


if __name__ == "__main__":
    main()
