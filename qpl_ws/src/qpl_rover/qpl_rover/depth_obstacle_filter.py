"""
Depth image -> Nav2 obstacle clouds.

Nav2's own height filter measures z in the costmap's global frame, which assumes
the floor is at z = 0 everywhere. Nothing measures the rover's z, so on the
terrain world a crest ahead of a rover sitting in a dip crosses the threshold
and is marked, and floor hits are dropped before raytracing so nothing ever
clears. This node does the height reasoning instead and hands Nav2 two
pre-classified, flattened clouds:

  <camera>/obstacles  points that are obstacles      (Nav2: marking only)
  <camera>/clearing   every valid return, incl. floor (Nav2: clearing only)

Heights are measured in a gravity-levelled frame centred on the rover: points go
into `level_frame` (odom, whose roll/pitch come from the IMU via the EKF), then
the rover's own ground point (the base_footprint origin) is subtracted. The arena
floor is bounded (+-7 cm on the terrain world), so in a level frame the floor
anywhere stays within ~0.14 m of the floor under the rover, even when the rover
is parked on a slope. A body-fixed frame would not: a 6 deg tilt puts floor 4 m
away 0.42 m off.

The obstacle threshold grows with range to absorb IMU tilt error (a 2 deg error
moves a point 4 m away by 0.14 m):

  min_height(r) = min_obstacle_height + r * height_slope_per_m

Walls (0.5 m in the sim) stay well above that at every range.

That rule alone misses the rulebook obstacles: a 15-30 cm boulder sitting in a
dip, or a crater (which is below the floor, not above it). So there is a second,
local test. The hills are smooth (within ~2 cm of their own 1 m average) while
boulders and craters are ~0.3-0.5 m across, so each point is also compared with
the *local* floor around it:

  1. points are binned into `local_cell` squares in the level frame and each
     cell's median height is taken, leaving out cells holding any point the
     first rule already marked (the base of a wall would otherwise count as
     floor and lift the estimate beside it)
  2. the local floor of a cell is the median of the cell medians within
     `local_window` of it; a boulder or crater fills only a few of those cells,
     so it doesn't drag the floor estimate with it
  3. a point is a bump if it is more than bump_height(r) above its local floor,
     and a dip (crater) if it is more than dip_depth(r) below it:

       bump_height(r) = min_bump_height + r * local_slope_per_m
       dip_depth(r)   = min_dip_depth   + r * local_slope_per_m

  4. a cell only counts if at least `min_cell_points` of its points are flagged,
     so single noisy returns don't mark anything

A crater shows up as its far inner wall, which the camera sees below the
surrounding floor. Dip points are marked where they are, inside the crater.
From 0.35 m up the camera sees little of a crater's inside, so within ~2 m the
floor hidden behind its near rim is marked too (crater_shadows()).

Boulder and crater marks are remembered (ObstacleMemory) so they survive
Nav2's raytracing once they are too close for the camera to see.

The cameras only see the near face of a boulder, and DWB only rejects a
trajectory when the footprint's edge crosses a marked cell, so in the sim the
rover brushed the unseen backs and sides of boulders. Detections are grown
before they are remembered (grow_detections()): bump points are extended away
from the camera by up to grow_behind_m (the rulebook's boulders are at most
0.40 m across), and every detection gets a grow_radius_m ring.

Both outputs are in `level_frame` with z set to the sensor's z, so Nav2's 3D
range checks act as 2D ones and its height band (set wide) never rejects them.
Nav2 must be given `sensor_frame` for these sources, since the cloud's own frame
(odom) is not where the rays start.

Input is the depth image, not the driver's point cloud: the sim clouds are
10-13 MB a frame and were dropped in transport for seconds at a time (Nav2 then
reuses the last cloud, from wherever the rover was). The image is ~8x smaller and
is deprojected here at the decimated resolution only.

Nothing lower than ignore_max_height is marked inside the arena zones named in
ignore_zones (the berm by default), so the rover's own dump piles don't block
the next dump slot. The rulebook lets the rover push obstacles aside in the
construction zone. Those points still clear. Needs ignore_frame (map) -> odom;
without it nothing is ignored.
"""
import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
import tf2_ros


def quaternion_matrix(q):
    """3x3 rotation matrix for a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_parts(tf):
    t = tf.transform.translation
    return quaternion_matrix(tf.transform.rotation), np.array([t.x, t.y, t.z])


def deproject(image, info, stride):
    """(rows, cols, 3) optical-frame points from every `stride`th depth pixel,
    NaN where there is no return. Kept as a grid so neighbouring rows can be
    compared (crater shadows)."""
    if image.encoding == '16UC1':
        depth = np.frombuffer(image.data, dtype=np.uint16).reshape(image.height, -1)[:, :image.width]
        scale = 0.001  # mm
    elif image.encoding == '32FC1':
        depth = np.frombuffer(image.data, dtype=np.float32).reshape(image.height, -1)[:, :image.width]
        scale = 1.0
    else:
        raise ValueError(f'unsupported depth encoding {image.encoding}')

    # camera_info can be for a different resolution than the image (RealSense decimation).
    sx = image.width / info.width
    sy = image.height / info.height
    fx, fy = info.k[0] * sx, info.k[4] * sy
    cx, cy = info.k[2] * sx, info.k[5] * sy

    v, u = np.mgrid[0:image.height:stride, 0:image.width:stride]
    z = depth[::stride, ::stride].astype(np.float64) * scale
    z[~(np.isfinite(z) & (z > 0))] = np.nan
    return np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=-1)


XYZ_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


def cell_groups(xy, cell):
    """Integer cell index per point, plus (unique cells, inverse index)."""
    ij = np.floor(xy / cell).astype(np.int64)
    cells, inverse = np.unique(ij, axis=0, return_inverse=True)
    return cells, inverse.ravel()


def group_median(values, inverse, n_groups):
    """Median of `values` within each group."""
    order = np.lexsort((values, inverse))
    v, g = values[order], inverse[order]
    counts = np.bincount(g, minlength=n_groups)
    starts = np.concatenate(([0], np.cumsum(counts)[:-1]))
    lo = starts + (counts - 1) // 2
    hi = starts + counts // 2
    return 0.5 * (v[lo] + v[hi])


def local_anomalies(xy, height, rng, floor_ok, cell, window, min_cells,
                    bump_height, dip_depth, slope_per_m, min_cell_points):
    """(flagged, rel): boolean mask of points that stick up from, or sink below,
    their local floor, and each point's height above that floor (NaN if unknown).

    xy and height are in the gravity-levelled frame; rng is horizontal range from
    the sensor. Cells holding any point with floor_ok unset (already an
    obstacle, e.g. a wall) are left out of the floor estimate, so the base of
    a wall doesn't count as floor. See the module docstring for the method."""
    flagged = np.zeros(len(height), dtype=bool)
    rel = np.full(len(height), np.nan)
    if len(height) == 0:
        return flagged, rel
    cells, inverse = cell_groups(xy, cell)
    blocked = np.bincount(inverse[~floor_ok], minlength=len(cells)) > 0
    floor_ok = floor_ok & ~blocked[inverse]
    if not floor_ok.any():
        return flagged, rel
    medians = np.full(len(cells), np.nan)
    used = np.unique(inverse[floor_ok])
    remap = np.full(len(cells), -1)
    remap[used] = np.arange(len(used))
    medians[used] = group_median(height[floor_ok], remap[inverse[floor_ok]], len(used))

    # Dense grid of cell medians (NaN where no points), then the median over a
    # (2k+1)^2 window around every cell.
    origin = cells.min(axis=0)
    idx = cells - origin
    k = max(1, int(round(window / cell)))
    grid = np.full(tuple(idx.max(axis=0) + 1 + 2 * k), np.nan)
    grid[idx[:, 0] + k, idx[:, 1] + k] = medians
    windows = np.lib.stride_tricks.sliding_window_view(grid, (2 * k + 1, 2 * k + 1))
    windows = windows[idx[:, 0], idx[:, 1]].reshape(len(cells), -1)
    enough = np.sum(~np.isnan(windows), axis=1) >= min_cells
    floor = np.full(len(cells), np.nan)
    if enough.any():
        floor[enough] = np.nanmedian(windows[enough], axis=1)

    rel = height - floor[inverse]          # NaN where there is no floor estimate
    with np.errstate(invalid='ignore'):
        raw = ((rel > bump_height + rng * slope_per_m)
               | (rel < -(dip_depth + rng * slope_per_m)))
    per_cell = np.bincount(inverse[raw], minlength=len(cells))
    return raw & (per_cell[inverse] >= min_cell_points), rel


def crater_shadows(grid, rel, sensor_pos, usable, blocked, row_angle,
                   max_range, min_gap, gap_ratio, max_gap, min_depth, fill_step):
    """Points filling the floor hidden behind a crater's near rim.

    grid is (rows, cols, 3) in the level frame, row 0 at the top of the image.
    Going up a column, horizontal range grows smoothly over a smooth floor: by
    about r^2 / h * row_angle per row, for a sensor h above the point. Close
    in, the sensor looks down more steeply than any arena slope, so nothing on
    a smooth floor is hidden. A jump much bigger than that, where the far
    point is not higher than the near one, means the floor dropped out of
    sight: a crater. The first point seen past the gap must also be at least
    min_depth below its local floor (rel, same grid shape): that is the
    crater's far inner wall. Without that test the floor sloping away behind a
    crater's raised rim would count too. Jumps whose near point is `blocked`
    (already an obstacle, e.g. a boulder hiding the floor behind it) are
    skipped."""
    near, far = grid[1:], grid[:-1]
    ok = usable[1:] & usable[:-1] & ~blocked[1:]
    r_near = np.hypot(near[..., 0] - sensor_pos[0], near[..., 1] - sensor_pos[1])
    r_far = np.hypot(far[..., 0] - sensor_pos[0], far[..., 1] - sensor_pos[1])
    h = np.maximum(sensor_pos[2] - near[..., 2], 0.05)
    with np.errstate(invalid='ignore'):
        gap = r_far - r_near
        expected = r_near ** 2 / h * row_angle
        hole = (ok & (r_near <= max_range) & (gap > np.maximum(min_gap, gap_ratio * expected))
                & (gap < max_gap) & (far[..., 2] <= near[..., 2] + 0.01)
                & (rel[:-1] < -min_depth))
    if not hole.any():
        return np.empty((0, 3))
    a, b = near[hole], far[hole]
    steps = np.arange(fill_step, max_gap, fill_step)
    t = steps[None, :] / gap[hole][:, None]
    fill = a[:, None, :] + t[..., None] * (b - a)[:, None, :]
    return np.concatenate([fill[t <= 1.0], b])


def grow_detections(pts, rel, sensor_pos, behind, radius, step):
    """Extra obstacle points around local detections, in the level frame: bumps
    (rel > 0) extended away from the sensor by up to `behind`, then a ring of
    `radius` around those and every detection. One point per `step` cell."""
    if len(pts) == 0 or (behind <= 0 and radius <= 0):
        return np.empty((0, 3))
    extra = []
    with np.errstate(invalid='ignore'):
        bumps = pts[rel > 0]
    if behind > 0 and len(bumps):
        away = bumps[:, :2] - sensor_pos[:2]
        away /= np.maximum(np.linalg.norm(away, axis=1, keepdims=True), 1e-6)
        k = np.arange(step, behind + 1e-9, step)
        xy = (bumps[:, None, :2] + away[:, None, :] * k[None, :, None]).reshape(-1, 2)
        extra.append(np.column_stack([xy, np.repeat(bumps[:, 2], len(k))]))
    if radius > 0:
        base = np.concatenate([pts] + extra)
        angle = np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False)
        ring = radius * np.column_stack([np.cos(angle), np.sin(angle)])
        xy = (base[:, None, :2] + ring[None]).reshape(-1, 2)
        extra.append(np.column_stack([xy, np.repeat(base[:, 2], len(angle))]))
    if not extra:
        return np.empty((0, 3))
    grown = np.concatenate(extra)
    _, first = np.unique(np.floor(grown[:, :2] / step).astype(np.int64), axis=0, return_index=True)
    return grown[first]


def load_zones(names):
    """(x_min, x_max, y_min, y_max) of each named zone in the selected arena config."""
    arena_dir = os.path.join(get_package_share_directory('qpl_rover'), 'config', 'arena')
    with open(os.path.join(arena_dir, 'selector.yaml')) as f:
        arena = yaml.safe_load(f)['arena']
    with open(os.path.join(arena_dir, f'{arena}.yaml')) as f:
        zones = {z['name']: (z['x_min'], z['x_max'], z['y_min'], z['y_max'])
                 for z in yaml.safe_load(f)['arena']['zones']}
    missing = [n for n in names if n not in zones]
    if missing:
        raise ValueError(f'arena {arena} has no zone {missing}; it has {sorted(zones)}')
    return [zones[n] for n in names]


def classify(p, grid, sensor_pos, base_pos, base_rot, row_angle, ignored=None):
    """Split a levelled point grid into Nav2 clouds. p holds the node's
    parameters (attributes of the same names). Returns (points, clearing,
    obstacle, local, rel): points (N, 3) still at true height, boolean masks
    for the clearing cloud, the obstacle cloud and the obstacles found by the
    local tests (boulders, craters), and each point's height above its local
    floor (NaN where unknown). Points flagged in `ignored` (per grid point) are
    left out of the local tests, so a dump pile doesn't raise the floor estimate
    around it; the caller removes their marks."""
    flat_grid = grid.reshape(-1, 3)
    finite = np.isfinite(flat_grid[:, 2])
    if ignored is None:
        ignored = np.zeros(len(flat_grid), bool)

    # Drop the rover itself (box in base_frame).
    in_base = (flat_grid - base_pos) @ base_rot
    rover = (np.abs(in_base[:, 0]) < p.self_filter_x) & (np.abs(in_base[:, 1]) < p.self_filter_y)

    with np.errstate(invalid='ignore'):
        rng = np.hypot(flat_grid[:, 0] - sensor_pos[0], flat_grid[:, 1] - sensor_pos[1])
        height = flat_grid[:, 2] - base_pos[2]
        clearing = (finite & ~rover & (rng >= p.min_range) & (rng <= p.clearing_max_range)
                    & (height <= p.max_obstacle_height))
    obstacle = np.zeros_like(clearing)
    obstacle[clearing] = (
        (rng[clearing] <= p.obstacle_max_range)
        & (height[clearing] >= p.min_obstacle_height + rng[clearing] * p.height_slope_per_m))

    extra = np.empty((0, 3))
    rel = np.full(len(flat_grid), np.nan)
    local = np.zeros_like(clearing)
    if p.local_enabled:
        near = clearing & (rng <= p.local_max_range) & ~ignored
        flagged, rel[near] = local_anomalies(
            flat_grid[near, :2], height[near], rng[near], ~obstacle[near],
            p.local_cell, p.local_window, p.local_min_cells,
            p.min_bump_height, p.min_dip_depth, p.local_slope_per_m,
            p.min_cell_points)
        local[near] = flagged & ~obstacle[near]
        obstacle[near] |= flagged
        shape = grid.shape[:2]
        extra = crater_shadows(
            grid, rel.reshape(shape), sensor_pos, (clearing & ~ignored).reshape(shape), obstacle.reshape(shape),
            row_angle, p.shadow_max_range, p.shadow_min_gap, p.shadow_gap_ratio,
            p.shadow_max_gap, p.shadow_min_depth, p.shadow_fill_step)

    points = np.concatenate([flat_grid[clearing], extra])
    n = clearing.sum()
    yes, no = np.ones(len(extra), bool), np.zeros(len(extra), bool)
    return (points,
            np.concatenate([np.ones(n, bool), no]),
            np.concatenate([obstacle[clearing], yes]),
            np.concatenate([local[clearing], yes]),
            np.concatenate([rel[clearing], np.full(len(extra), np.nan)]))


class ObstacleMemory:
    """Boulder and crater detections, kept in the level frame and republished
    every frame.

    Nav2 clears costmap cells along every ray in 2D, whatever the height, so
    rays to the floor beyond a crater pass over it and wipe it out on any frame
    it isn't detected, and for good once it is too close for the camera to see
    (a level camera 0.35 m up sees no floor closer than ~0.85 m). A remembered
    cell is kept while it is out of sight, however long the rover waits there,
    and dropped once it is seen again as plain floor. `duration` seconds after
    its last detection it is dropped anyway, so localisation error can't leave
    stale marks around for ever. The node keeps it in memory_frame (map)."""

    def __init__(self, cell, duration, floor_band, min_floor_points):
        self.cell, self.duration = cell, duration
        self.floor_band, self.min_floor_points = floor_band, min_floor_points
        self.cells = {}                       # (i, j) -> (x, y, z, last seen)

    def update(self, now, points, clearing, obstacle, local, rel):
        ij = np.floor(points[:, :2] / self.cell).astype(np.int64)
        # forget cells now seen as plain floor: enough near-floor returns, no obstacle
        with np.errstate(invalid='ignore'):
            floor = clearing & ~obstacle & (np.abs(rel) < self.floor_band)
        if self.cells and floor.any():
            cells, counts = np.unique(ij[floor], axis=0, return_counts=True)
            blocked = {tuple(c) for c in ij[obstacle]}
            for c, n in zip(map(tuple, cells), counts):
                if n >= self.min_floor_points and c not in blocked:
                    self.cells.pop(c, None)
        for c, pt in zip(map(tuple, ij[local]), points[local]):
            self.cells[c] = (pt[0], pt[1], pt[2], now)
        for c in [c for c, v in self.cells.items() if now - v[3] > self.duration]:
            del self.cells[c]
        if not self.cells:
            return np.empty((0, 3))
        return np.array([v[:3] for v in self.cells.values()])


def make_cloud(header, xyz):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(xyz)
    msg.fields = XYZ_FIELDS
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(xyz)
    msg.is_dense = True
    msg.data = xyz.astype(np.float32).tobytes()
    return msg


class DepthObstacleFilter(Node):
    def __init__(self):
        super().__init__('depth_obstacle_filter')

        self.depth_topic = self.declare_parameter('depth_topic', '/depth_camera_front/depth/image_raw').value
        self.info_topic = self.declare_parameter('info_topic', '/depth_camera_front/depth/camera_info').value
        self.output_namespace = self.declare_parameter('output_namespace', '/depth_camera_front').value
        self.level_frame = self.declare_parameter('level_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value

        # Decimation and rate: the Orbbec runs 640x480 at 30 Hz, and the RealSense is already
        # decimated 4x on the rover but not in the sim, so decimate to a width, not by a factor.
        self.target_width = int(self.declare_parameter('target_width', 160).value)
        self.max_rate = float(self.declare_parameter('max_rate', 10.0).value)

        # Ranges are horizontal distance from the sensor.
        self.min_range = self.declare_parameter('min_range', 0.30).value
        self.obstacle_max_range = self.declare_parameter('obstacle_max_range', 4.0).value
        self.clearing_max_range = self.declare_parameter('clearing_max_range', 5.0).value

        # Height above the floor under the rover.
        self.min_obstacle_height = self.declare_parameter('min_obstacle_height', 0.16).value
        self.height_slope_per_m = self.declare_parameter('height_slope_per_m', 0.035).value
        self.max_obstacle_height = self.declare_parameter('max_obstacle_height', 1.0).value

        # Local test for boulders and craters: height relative to the floor around each point.
        self.local_enabled = self.declare_parameter('local_enabled', True).value
        self.local_max_range = self.declare_parameter('local_max_range', 3.5).value
        self.local_cell = self.declare_parameter('local_cell', 0.15).value
        self.local_window = self.declare_parameter('local_window', 0.6).value
        self.local_min_cells = int(self.declare_parameter('local_min_cells', 6).value)
        self.min_bump_height = self.declare_parameter('min_bump_height', 0.07).value
        self.min_dip_depth = self.declare_parameter('min_dip_depth', 0.06).value
        self.local_slope_per_m = self.declare_parameter('local_slope_per_m', 0.01).value
        self.min_cell_points = int(self.declare_parameter('min_cell_points', 2).value)
        # Crater shadows (floor hidden behind a crater's near rim), see crater_shadows().
        self.shadow_max_range = self.declare_parameter('shadow_max_range', 2.0).value
        self.shadow_min_gap = self.declare_parameter('shadow_min_gap', 0.15).value
        self.shadow_gap_ratio = self.declare_parameter('shadow_gap_ratio', 4.0).value
        self.shadow_max_gap = self.declare_parameter('shadow_max_gap', 0.7).value
        self.shadow_min_depth = self.declare_parameter('shadow_min_depth', 0.02).value
        self.shadow_fill_step = self.declare_parameter('shadow_fill_step', 0.05).value
        self.memory = ObstacleMemory(
            self.declare_parameter('memory_cell', 0.05).value,
            self.declare_parameter('memory_duration', 60.0).value,
            self.declare_parameter('memory_floor_band', 0.03).value,
            int(self.declare_parameter('memory_floor_points', 3).value))
        # Remembered marks are kept in this frame. map, not the level frame (odom): odom drifts,
        # so marks kept for minutes (a 360 in teleop maps the arena before AUTO) would end up
        # beside the obstacles. Until map exists nothing is remembered.
        self.memory_frame = self.declare_parameter('memory_frame', 'map').value

        # Growth of boulder/crater detections (see grow_detections()).
        self.grow_behind_m = self.declare_parameter('grow_behind_m', 0.30).value
        self.grow_radius_m = self.declare_parameter('grow_radius_m', 0.05).value
        self.grow_step_m = self.declare_parameter('grow_step_m', 0.05).value

        # Points inside this base_frame box are the rover itself.
        self.self_filter_x = self.declare_parameter('self_filter_x', 0.65).value
        self.self_filter_y = self.declare_parameter('self_filter_y', 0.56).value

        # Arena zones where nothing below ignore_max_height is marked (see the module docstring).
        self.ignore_enabled = self.declare_parameter('ignore_enabled', True).value
        self.ignore_zones = list(self.declare_parameter('ignore_zones', ['BERM']).value)
        self.ignore_frame = self.declare_parameter('ignore_frame', 'map').value
        self.ignore_max_height = self.declare_parameter('ignore_max_height', 0.6).value
        self.ignore_rects = load_zones(self.ignore_zones) if self.ignore_enabled else []

        self.tf_buffer = tf2_ros.Buffer()
        # No spin_thread: it adds this same node to a second executor, which
        # rclpy.spin() then takes it back from, leaving /tf to the thread that
        # blocks in lookup(). main() spins multi-threaded instead; the listener's
        # callbacks are in a reentrant group so they run while on_depth waits.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.obstacle_pub = self.create_publisher(
            PointCloud2, f'{self.output_namespace}/obstacles', qos_profile_sensor_data)
        self.clearing_pub = self.create_publisher(
            PointCloud2, f'{self.output_namespace}/clearing', qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)

        self.info = None
        self.last_stamp = None
        self.get_logger().info(
            f'{self.depth_topic} -> {self.output_namespace}/{{obstacles,clearing}} in {self.level_frame}')

    def on_info(self, msg):
        self.info = msg

    def lookup(self, target, source, stamp):
        """Transform at `stamp`; if TF hasn't caught up yet, the latest one if it's within 0.1 s."""
        try:
            return self.tf_buffer.lookup_transform(target, source, stamp, Duration(seconds=0.1))
        except tf2_ros.ExtrapolationException:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
            if abs((Time.from_msg(tf.header.stamp) - stamp).nanoseconds) > 1e8:
                raise
            return tf

    def remember(self, stamp, points, clearing, obstacle, local, rel):
        """Update the obstacle memory with this frame's level-frame points; the remembered
        marks back in the level frame ((0, 3) without a memory_frame transform yet)."""
        if self.memory_frame == self.level_frame:
            return self.memory.update(stamp.nanoseconds * 1e-9, points, clearing, obstacle, local, rel)
        try:
            # Latest, not at stamp: map -> odom only moves on a localisation correction
            tf = self.tf_buffer.lookup_transform(self.memory_frame, self.level_frame, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'No {self.memory_frame} frame, so nothing is remembered: {e}',
                                   throttle_duration_sec=10.0)
            return np.empty((0, 3))
        rot, pos = transform_parts(tf)
        remembered = self.memory.update(stamp.nanoseconds * 1e-9, points @ rot.T + pos,
                                        clearing, obstacle, local, rel)
        return (remembered - pos) @ rot if len(remembered) else remembered

    def ignore_test(self, base_pos):
        """A function (N, 3) level-frame points -> mask of those inside an ignore zone and below
        ignore_max_height, or None when nothing can be ignored (no zones, or no map frame yet)."""
        if not self.ignore_rects:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.ignore_frame, self.level_frame, Time())
        except tf2_ros.TransformException:
            return None
        rot, pos = transform_parts(tf)

        def test(pts):
            xy = (pts @ rot.T + pos)[:, :2]
            inside = np.zeros(len(pts), bool)
            with np.errstate(invalid='ignore'):
                for x_min, x_max, y_min, y_max in self.ignore_rects:
                    inside |= (xy[:, 0] >= x_min) & (xy[:, 0] <= x_max) & (xy[:, 1] >= y_min) & (xy[:, 1] <= y_max)
                return inside & (pts[:, 2] - base_pos[2] < self.ignore_max_height)
        return test

    def on_depth(self, msg):
        if self.info is None:
            self.get_logger().warn(f'No camera_info on {self.info_topic} yet', throttle_duration_sec=5.0)
            return

        stamp = Time.from_msg(msg.header.stamp)
        # A stamp going backwards (sim reset) resets the throttle and the memory.
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.memory.cells.clear()
        if self.last_stamp is not None and 0 <= (stamp - self.last_stamp).nanoseconds < 1e9 / self.max_rate:
            return
        self.last_stamp = stamp

        try:
            sensor_tf = self.lookup(self.level_frame, msg.header.frame_id, stamp)
            base_tf = self.lookup(self.level_frame, self.base_frame, stamp)
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return

        sensor_rot, sensor_pos = transform_parts(sensor_tf)
        base_rot, base_pos = transform_parts(base_tf)

        stride = max(1, msg.width // self.target_width)
        grid = deproject(msg, self.info, stride) @ sensor_rot.T + sensor_pos
        # angle between decimated rows (camera_info may be for another resolution)
        row_angle = stride / (self.info.k[4] * msg.height / self.info.height)
        ignore = self.ignore_test(base_pos)
        points, clearing, obstacle, local, rel = classify(
            self, grid, sensor_pos, base_pos, base_rot, row_angle,
            ignore(grid.reshape(-1, 3)) if ignore else None)
        grown = grow_detections(points[local], rel[local], sensor_pos,
                                self.grow_behind_m, self.grow_radius_m, self.grow_step_m)
        if len(grown):
            n = len(grown)
            points = np.concatenate([points, grown])
            clearing = np.concatenate([clearing, np.zeros(n, bool)])
            obstacle = np.concatenate([obstacle, np.ones(n, bool)])
            local = np.concatenate([local, np.ones(n, bool)])
            rel = np.concatenate([rel, np.full(n, np.nan)])
        if ignore:
            dropped = ignore(points)
            obstacle &= ~dropped
            local &= ~dropped
        remembered = self.remember(stamp, points, clearing, obstacle, local, rel)
        if ignore and len(remembered):
            remembered = remembered[~ignore(remembered)]

        flat = np.concatenate([points[obstacle], remembered])
        # Nothing under the rover itself: the cameras can't see there, so a remembered mark the
        # rover has edged onto (a crater rim it grazed) was republished every frame, and with
        # its footprint "in collision" Nav2 rejected every trajectory, spin and backup, and it
        # never moved again. The marks stay in memory and come back once it has moved off.
        in_base = (flat - base_pos) @ base_rot
        flat = flat[~((np.abs(in_base[:, 0]) < self.self_filter_x) & (np.abs(in_base[:, 1]) < self.self_filter_y))]
        flat[:, 2] = sensor_pos[2]
        clear = points[clearing]
        clear[:, 2] = sensor_pos[2]

        header = msg.header
        header.frame_id = self.level_frame
        self.obstacle_pub.publish(make_cloud(header, flat))
        self.clearing_pub.publish(make_cloud(header, clear))


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleFilter()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the worker threads first or destroy_node() waits on them.
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
