"""
Random craters and boulders for the US arena sim (us_arena_terrain.world).

sim.launch.py calls build_world() on every launch, so each run gets a new
obstacle field, like the competition, where the judges re-place obstacles
before every attempt. Layout follows the 2026 Lunabotics Guidebook v3, 3.1
and Figure 2:

  - at least 3 boulders in the obstacle zone, ~30-40 cm diameter, random
    heights; boulders may also be in the excavation zone, no bigger than the
    obstacle-zone ones
  - at least 3 craters in the obstacle zone, no wider or deeper than 40-50 cm

Craters are carved into the floor mesh itself (generate_arena_terrain.py), so
they are part of the ground the wheels drive on. Boulders are separate static
rock meshes resting on that surface. The hills and texture stay the same as the
checked-in terrain; only the obstacles change.

Everything goes into a fresh temp dir: terrain mesh, one mesh per boulder,
the world file, and obstacles.yaml listing the seed and every obstacle.

Try it without Gazebo:  python3 worlds/arena_obstacles.py --seed 1
"""

import argparse
import os
import re
import sys
import tempfile

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_arena_terrain as terrain  # noqa: E402

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SOURCE_WORLD = os.path.join(BASE_DIR, "us_arena_terrain.world")
SAND_TEXTURE = os.path.join(BASE_DIR, "arena_terrain", "meshes", "sand_diffuse.jpg")

# -----------------------------
# Rulebook layout (arena coords, origin at the south-west corner)
# -----------------------------
EXCAVATION_ZONE = (0.0, 0.0, terrain.ARENA_X, 3.0)            # (x_min, y_min, x_max, y_max)
OBSTACLE_ZONE = (0.0, 3.0, terrain.ARENA_X, terrain.ARENA_Y)
KEEP_OUT = terrain.FLAT_ZONES                                  # starting zone, construction zone
KEEP_OUT_MARGIN = 0.5          # obstacles stay this far outside the keep-out zones (m)
WALL_MARGIN = terrain.BORDER   # and this far from the walls (m)

CRATERS = (3, 5)               # count in the obstacle zone (inclusive)
CRATER_DIAMETER = (0.40, 0.50)
CRATER_DEPTH = (0.10, 0.30)    # rim top to bowl floor (m)
CRATER_RIM = 0.2               # fraction of the depth that is rim above the ground; dug holes, not volcanoes

BOULDERS = (3, 5)              # count in the obstacle zone (inclusive)
EXCAVATION_BOULDERS = (0, 2)   # count in the excavation zone (inclusive)
BOULDER_DIAMETER = (0.30, 0.40)
BOULDER_HEIGHT = (0.15, 0.40)
BOULDER_SINK = 0.03            # pushed into the sand so the base never floats (m)

# Rover is 0.90 m wide: centres this far apart leave a gap it fits through,
# so there is always a slalom route.
MIN_SPACING = 1.3
MAX_TRIES = 2000


def _in_rect(x, y, rect, margin=0.0):
    x0, y0, x1, y1 = rect
    return x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin


def _sample_point(rng, zone, placed, radius):
    """Uniform point in zone whose footprint (radius, m) stays clear of the
    walls and keep-out zones, and away from already placed obstacles."""
    x0, y0, x1, y1 = zone
    wall = WALL_MARGIN + radius
    for _ in range(MAX_TRIES):
        x = rng.uniform(max(x0, wall), min(x1, terrain.ARENA_X - wall))
        y = rng.uniform(max(y0, wall), min(y1, terrain.ARENA_Y - wall))
        if any(_in_rect(x, y, r, KEEP_OUT_MARGIN + radius) for r in KEEP_OUT):
            continue
        if any(np.hypot(x - px, y - py) < MIN_SPACING for px, py in placed):
            continue
        return x, y
    raise RuntimeError("could not place an obstacle, too crowded; lower the counts or MIN_SPACING")


def place_obstacles(rng):
    """Returns (craters, boulders). Craters are dicts with x, y, diameter,
    depth; boulders with x, y, diameter, height, yaw, zone."""
    placed = []
    craters, boulders = [], []

    for _ in range(rng.integers(CRATERS[0], CRATERS[1] + 1)):
        diameter = rng.uniform(*CRATER_DIAMETER)
        # the raised rim reaches 1.45 crater radii (add_crater's rim_width)
        x, y = _sample_point(rng, OBSTACLE_ZONE, placed, 0.725 * diameter)
        placed.append((x, y))
        craters.append({"x": x, "y": y, "diameter": diameter,
                        "depth": rng.uniform(*CRATER_DEPTH)})

    def add_boulder(zone, name, max_d, max_h):
        diameter = rng.uniform(BOULDER_DIAMETER[0], max_d)
        x, y = _sample_point(rng, zone, placed, diameter / 2)
        placed.append((x, y))
        boulders.append({"x": x, "y": y, "zone": name, "diameter": diameter,
                         "height": rng.uniform(BOULDER_HEIGHT[0], max_h),
                         "yaw": rng.uniform(-np.pi, np.pi)})

    for _ in range(rng.integers(BOULDERS[0], BOULDERS[1] + 1)):
        add_boulder(OBSTACLE_ZONE, "obstacle", BOULDER_DIAMETER[1], BOULDER_HEIGHT[1])
    # excavation-zone boulders are no bigger than the smallest obstacle-zone one
    max_d = min(b["diameter"] for b in boulders)
    max_h = min(b["height"] for b in boulders)
    for _ in range(rng.integers(EXCAVATION_BOULDERS[0], EXCAVATION_BOULDERS[1] + 1)):
        add_boulder(EXCAVATION_ZONE, "excavation", max_d, max_h)

    return craters, boulders


def _icosphere(subdivisions=3):
    """Unit icosphere: (vertices, triangles)."""
    t = (1 + 5 ** 0.5) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t),
         (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4),
         (11, 10, 2), (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8),
         (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    verts = [np.array(p, float) / np.linalg.norm(p) for p in v]
    for _ in range(subdivisions):
        cache, faces = {}, []

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = verts[a] + verts[b]
                verts.append(m / np.linalg.norm(m))
                cache[key] = len(verts) - 1
            return cache[key]

        for a, b, c in f:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        f = faces
    return np.array(verts), np.array(f)


def rock_mesh(rng, diameter, height, path):
    """Write an irregular boulder OBJ, diameter wide (x/y) and height tall,
    base at z = 0: a lumpy icosphere with a few flat fracture faces and a
    flattened underside, flat-shaded so the facets read as broken rock."""
    v, f = _icosphere(3)
    # low-frequency lumps from random spherical "bumps"
    r = np.ones(len(v))
    for _ in range(10):
        d = rng.standard_normal(3)
        d /= np.linalg.norm(d)
        r += rng.uniform(-0.2, 0.2) * np.exp(-((1 - v @ d) / rng.uniform(0.2, 0.5)) ** 2)
    v = v * r[:, None]
    # fracture faces: flatten everything beyond a few random planes
    for _ in range(rng.integers(4, 8)):
        d = rng.standard_normal(3)
        d[2] = abs(d[2]) * 0.5                   # mostly on the sides and top
        d /= np.linalg.norm(d)
        excess = v @ d - rng.uniform(0.7, 0.9)
        v -= np.maximum(excess, 0)[:, None] * d
    v[:, 2] = np.maximum(v[:, 2], -0.55)         # flat underside, sits in the sand
    v -= v.min(axis=0)
    v *= np.array([diameter, diameter, height]) / v.max(axis=0)
    v[:, :2] -= np.array([diameter, diameter]) / 2

    # one normal per face (flat shading); vertices are shared, normals are not
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    fn /= np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-12)
    ni = np.arange(1, len(f) + 1)[:, None].repeat(3, axis=1)

    shade = rng.uniform(0.32, 0.45)              # darker than the sand
    with open(path, "w") as fh:
        fh.write("# Generated by worlds/arena_obstacles.py\n")
        fh.write(f"mtllib {os.path.basename(path)[:-4]}.mtl\nusemtl rock\n")
        np.savetxt(fh, v, fmt="v %.4f %.4f %.4f")
        np.savetxt(fh, fn, fmt="vn %.4f %.4f %.4f")
        np.savetxt(fh, np.stack([f + 1, ni], -1).reshape(-1, 6), fmt="f %d//%d %d//%d %d//%d")
    with open(path[:-4] + ".mtl", "w") as fh:
        fh.write(f"newmtl rock\nKa {shade:.3f} {shade:.3f} {shade:.3f}\n"
                 f"Kd {shade:.3f} {shade * 0.97:.3f} {shade * 0.93:.3f}\nKs 0 0 0\nillum 1\n")


def _mesh_model(name, mesh_uri, pose, friction=True):
    surface = ("""
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>"""
               if friction else "")
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>{surface}
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>
        </visual>
      </link>
    </model>
"""


def build_world(seed=None, out_dir=None, src_world=SOURCE_WORLD):
    """Generate a world with random obstacles. Returns (world_path, seed)."""
    if seed is None or seed < 0:
        seed = int(np.random.SeedSequence().entropy % 2**31)
    rng = np.random.default_rng(seed)
    out_dir = out_dir or tempfile.mkdtemp(prefix="qpl_arena_")
    mesh_dir = os.path.join(out_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)

    craters, boulders = place_obstacles(rng)

    h = terrain.make_heightmap([(c["x"], c["y"], c["diameter"], c["depth"], CRATER_RIM)
                                for c in craters])
    terrain.write_mesh(h, mesh_dir)
    os.symlink(SAND_TEXTURE, os.path.join(mesh_dir, "sand_diffuse.jpg"))  # terrain.mtl wants it beside the mesh
    models = _mesh_model("arena_terrain", "file://" + os.path.join(mesh_dir, "terrain.obj"),
                         "0 0 0 0 0 0")

    for i, b in enumerate(boulders):
        path = os.path.join(mesh_dir, f"boulder_{i}.obj")
        rock_mesh(rng, b["diameter"], b["height"], path)
        # sit on the lowest terrain point under the rock so no edge floats
        ring = np.linspace(0, 2 * np.pi, 12, endpoint=False)
        rx = b["x"] + np.r_[0, 0.5 * b["diameter"] * np.cos(ring)]
        ry = b["y"] + np.r_[0, 0.5 * b["diameter"] * np.sin(ring)]
        b["z"] = float(terrain.height_at(h, rx, ry).min()) - BOULDER_SINK
        models += _mesh_model(f"boulder_{i}", "file://" + path,
                              f"{b['x']:.3f} {b['y']:.3f} {b['z']:.3f} 0 0 {b['yaw']:.3f}",
                              friction=False)

    with open(src_world) as fh:
        world = fh.read()
    world, n = re.subn(r"\s*<include>\s*<uri>model://arena_terrain</uri>.*?</include>",
                       "", world, flags=re.S)
    if n != 1:
        raise RuntimeError(f"{src_world}: expected one model://arena_terrain include, found {n}")
    world = world.replace("</world>", f"    <!-- Generated by arena_obstacles.py, seed {seed} -->"
                                      f"{models}\n  </world>", 1)
    world_path = os.path.join(out_dir, "us_arena_obstacles.world")
    with open(world_path, "w") as fh:
        fh.write(world)

    def rounded(items):
        return [{k: (round(float(v), 3) if not isinstance(v, str) else v) for k, v in it.items()}
                for it in items]

    with open(os.path.join(out_dir, "obstacles.yaml"), "w") as fh:
        yaml.safe_dump({"seed": seed, "craters": rounded(craters), "boulders": rounded(boulders)},
                       fh, sort_keys=False)
    return world_path, seed


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--seed", type=int, default=-1, help="-1 for random")
    parser.add_argument("--out", help="output dir (default: new temp dir)")
    args = parser.parse_args()
    world_path, seed = build_world(args.seed, args.out)
    print(f"seed {seed}\nworld {world_path}")


if __name__ == "__main__":
    main()
