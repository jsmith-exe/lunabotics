"""Widest route through a Nav2 costmap: the way round that keeps furthest from lethal cells.

Nav2's planner takes the shortest way that fits, which in a cluster of craters and boulders can be
a gap the rover then can't get through; nav_autonomy_node falls back on this after a failed leg."""
import heapq
from math import hypot

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - scipy fallback
    cv2 = None
    from scipy import ndimage

LETHAL = 100  # OccupancyGrid value of a lethal cell
NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def clearance_map(grid):
    """(rows, cols) metres from each cell to the nearest lethal cell. Unknown counts as free."""
    info = grid.info
    cells = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)
    free = (cells != LETHAL).astype(np.uint8)
    if cv2 is not None:
        dist = cv2.distanceTransform(free, cv2.DIST_L2, 5)
    else:
        dist = ndimage.distance_transform_edt(free)
    return dist * info.resolution


def _search(dist, start, goal, floor):
    # Shortest 8-connected path from start to goal through cells with dist >= floor, or None
    rows, cols = dist.shape
    best = {start: 0.0}
    prev = {}
    heap = [(0.0, start)]
    while heap:
        d, cell = heapq.heappop(heap)
        if cell == goal:
            path = [cell]
            while path[-1] in prev:
                path.append(prev[path[-1]])
            return path[::-1]
        if d > best[cell]:
            continue
        r, c = cell
        for dr, dc in NEIGHBOURS:
            n = (r + dr, c + dc)
            if 0 <= n[0] < rows and 0 <= n[1] < cols and dist[n] >= floor:
                nd = d + (1.4142 if dr and dc else 1.0)
                if nd < best.get(n, float("inf")):
                    best[n] = nd
                    prev[n] = cell
                    heapq.heappush(heap, (nd, n))
    return None


def _connected(clear, a, b):
    # Whether cells a and b are in one 8-connected region of `clear`
    if cv2 is not None:
        _, labels = cv2.connectedComponents(clear.astype(np.uint8), connectivity=8)
    else:
        labels, _ = ndimage.label(clear, structure=np.ones((3, 3)))
    return labels[a] != 0 and labels[a] == labels[b]


def _bottleneck(dist, start, goal):
    # Largest clearance a path from start to goal can keep all the way (maximin): the highest
    # clearance level at which both are still in one connected region of cells at least that clear.
    # Every cell is in that region at the lowest level, and none past the lower of the two ends.
    levels = np.unique(dist)
    levels = levels[levels <= min(dist[start], dist[goal])]
    lo, hi = 0, len(levels) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if _connected(dist >= levels[mid], start, goal):
            lo = mid
        else:
            hi = mid - 1
    return levels[lo]


def widest_route(grid, start_xy, goal_xy, min_clearance, spacing=1.5, end_gap=1.0):
    """(waypoints, clearance, path): map-frame (x, y) points about `spacing` apart along the
    shortest of the widest routes from start_xy to goal_xy, leaving out the last `end_gap` m
    before the goal; the clearance it keeps from lethal cells; and the whole route, cell by cell.
    ([], clearance, []) when it is under min_clearance."""
    info = grid.info
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y

    def cell(x, y):
        return (min(max(int((y - oy) / res), 0), info.height - 1),
                min(max(int((x - ox) / res), 0), info.width - 1))

    dist = clearance_map(grid)
    start, goal = cell(*start_xy), cell(*goal_xy)
    # The ends may sit close to obstacles (a dig spot by a boulder); only the way between matters
    dist[start], dist[goal] = max(dist[start], 1e3), max(dist[goal], 1e3)
    clearance = _bottleneck(dist, start, goal)
    if clearance < min_clearance:
        return [], clearance, []
    path = _search(dist, start, goal, clearance - 0.05)
    if not path:
        return [], clearance, []
    points = [(ox + (c + 0.5) * res, oy + (r + 0.5) * res) for r, c in path]
    waypoints, travelled, last = [], 0.0, points[0]
    for p in points[1:]:
        travelled += hypot(p[0] - last[0], p[1] - last[1])
        last = p
        if travelled >= spacing and hypot(goal_xy[0] - p[0], goal_xy[1] - p[1]) > end_gap:
            waypoints.append(p)
            travelled = 0.0
    return waypoints, clearance, points


def path_clearance(grid, points, step=0.05):
    """Smallest clearance from lethal cells along the polyline through map-frame (x, y) points."""
    info = grid.info
    dist = clearance_map(grid)
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    best = float("inf")
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        n = max(1, int(hypot(x1 - x0, y1 - y0) / step))
        for k in range(n + 1):
            x, y = x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n
            r = min(max(int((y - oy) / res), 0), info.height - 1)
            c = min(max(int((x - ox) / res), 0), info.width - 1)
            best = min(best, float(dist[r, c]))
    return best

