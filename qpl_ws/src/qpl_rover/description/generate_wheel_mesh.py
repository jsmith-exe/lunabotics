"""
Generate the sim wheel mesh (description/meshes/wheel.stl).

A wheel_r x wheel_w cylinder whose rim is rounded with a FILLET-radius arc
between the flat sides and the tread, like a tyre. Axis along y, centred on
the origin, in metres, so it drops straight into the wheel link with no
rotation. Used as the wheel visual in sim_mode (rover_core.xacro); the
collision is stacked cylinders following the same arc, because ODE has no
filleted shape and mesh-on-mesh contact against the terrain mesh is poor.

Run from anywhere:  python3 description/generate_wheel_mesh.py
Keep RADIUS, WIDTH and FILLET in step with wheel_r, wheel_w and tyre_fillet.
"""

import os
import struct

import numpy as np

RADIUS = 0.15
WIDTH = 0.15
FILLET = 0.04
AROUND = 96        # segments around the axis
FILLET_STEPS = 12  # segments along each rounded edge

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'meshes', 'wheel.stl')


def profile():
    """(r, y) points from the +y hub centre, over the rounded rim, to the -y hub centre."""
    half = WIDTH / 2
    points = [(0.0, half)]
    # +y edge: from the flat side (pointing +y) round to the tread (pointing +r).
    for a in np.linspace(np.pi / 2, 0, FILLET_STEPS + 1):
        points.append((RADIUS - FILLET + FILLET * np.cos(a), half - FILLET + FILLET * np.sin(a)))
    # -y edge, mirrored.
    for a in np.linspace(0, -np.pi / 2, FILLET_STEPS + 1):
        points.append((RADIUS - FILLET + FILLET * np.cos(a), -half + FILLET + FILLET * np.sin(a)))
    points.append((0.0, -half))
    return points


def triangles():
    ring = profile()
    angles = np.linspace(0, 2 * np.pi, AROUND + 1)

    def vertex(r, y, theta):
        return np.array([r * np.cos(theta), y, r * np.sin(theta)])

    tris = []
    for i in range(len(ring) - 1):
        (r0, y0), (r1, y1) = ring[i], ring[i + 1]
        for j in range(AROUND):
            t0, t1 = angles[j], angles[j + 1]
            a, b = vertex(r0, y0, t0), vertex(r0, y0, t1)
            c, d = vertex(r1, y1, t0), vertex(r1, y1, t1)
            if r0 > 0:
                tris.append((a, b, c))
            if r1 > 0:
                tris.append((b, d, c))
    return tris


def write_stl(path, tris):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(b'qpl_rover wheel, rounded rim'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            n = np.cross(b - a, c - a)
            n /= np.linalg.norm(n)
            f.write(struct.pack('<12fH', *n, *a, *b, *c, 0))


if __name__ == '__main__':
    tris = triangles()
    write_stl(OUT, tris)
    print(f'{OUT}: {len(tris)} triangles, r={RADIUS} w={WIDTH} fillet={FILLET}')
