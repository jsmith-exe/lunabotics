"""Footprint checks against a Nav2 costmap published as an OccupancyGrid (0-100, -1 unknown)."""
from math import cos, sin

import numpy as np

LETHAL = 100


def footprint_blocked(grid, x, y, yaw, half_length, half_width, margin=0.0):
    """True if a lethal cell lies under the rover's rectangle, grown by margin, at (x, y, yaw).

    Unknown cells count as free, since the cameras may not have seen the spot yet. A rectangle
    that leaves the grid counts as blocked. Assumes the grid's origin isn't rotated, which
    holds for Nav2 costmaps.
    """
    info = grid.info
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    hl, hw = half_length + margin, half_width + margin
    c, s = cos(yaw), sin(yaw)
    xs = [x + c * a - s * b for a in (hl, -hl) for b in (hw, -hw)]
    ys = [y + s * a + c * b for a in (hl, -hl) for b in (hw, -hw)]
    i0, i1 = int(np.floor((min(xs) - ox) / res)), int(np.floor((max(xs) - ox) / res))
    j0, j1 = int(np.floor((min(ys) - oy) / res)), int(np.floor((max(ys) - oy) / res))
    if i0 < 0 or j0 < 0 or i1 >= info.width or j1 >= info.height:
        return True

    cells = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)[j0:j1 + 1, i0:i1 + 1]
    jj, ii = np.mgrid[j0:j1 + 1, i0:i1 + 1]
    dx = ox + (ii + 0.5) * res - x
    dy = oy + (jj + 0.5) * res - y
    inside = (np.abs(c * dx + s * dy) <= hl) & (np.abs(-s * dx + c * dy) <= hw)
    return bool(np.any(cells[inside] >= LETHAL))
