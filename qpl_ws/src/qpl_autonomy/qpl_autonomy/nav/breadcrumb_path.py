import os
from math import atan2, ceil, cos, hypot, sin

import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class BreadcrumbPath:
    """Recorder mode path: map-frame [x, y, yaw] breadcrumbs from the excavation zone (first)
    to the deposition point (last), dropped by the operator while teleoperating.
    """

    def __init__(self, points=None):
        self.points = [list(point) for point in points] if points else []

    """File"""
    @classmethod
    def load(cls, file):
        # Empty path if there's no file yet; raises on a malformed one
        if not os.path.exists(file):
            return cls()
        with open(file) as f:
            data = yaml.safe_load(f) or {}
        points = data.get("points", [])
        if any(len(point) != 3 for point in points):
            raise ValueError("every point must be [x, y, yaw]")
        return cls([[float(value) for value in point] for point in points])

    def save(self, file):
        # Write to a temp file and rename, so a crash mid-write can't leave half a path
        os.makedirs(os.path.dirname(file) or ".", exist_ok=True)
        tmp = f"{file}.tmp"
        with open(tmp, "w") as f:
            f.write("# Recorder mode breadcrumbs, map frame [x, y, yaw]: excavation zone first, deposition last\n")
            yaml.safe_dump({"points": [[round(value, 4) for value in point] for point in self.points]},
                           f, default_flow_style=None)
        os.replace(tmp, file)

    """Following"""
    def poses_toward(self, toward_deposition, from_x, from_y, spacing_m):
        # Dense (x, y, yaw) path from the rover through the rest of the breadcrumbs.
        # Joins at the end of the path segment nearest the rover, so it picks up the path
        # partway and never doubles back to a breadcrumb it's already next to. Ties go to the
        # later segment.
        points = self.points if toward_deposition else self.points[::-1]
        nearest = min(range(len(points) - 1),
                      key=lambda i: (segment_distance(from_x, from_y, points[i], points[i + 1]), -i))
        corners = [(from_x, from_y)] + [(x, y) for x, y, _ in points[nearest + 1:]]

        poses = []
        for (x0, y0), (x1, y1) in zip(corners, corners[1:]):
            length = hypot(x1 - x0, y1 - y0)
            if length < 1e-3:
                continue
            # Each pose faces along its segment; the controller only needs the positions
            heading = atan2(y1 - y0, x1 - x0)
            steps = max(1, ceil(length / spacing_m))
            for step in range(steps):
                t = step / steps
                poses.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0), heading))

        # Finish on the endpoint facing the way the operator left it
        end_x, end_y, end_yaw = points[-1]
        poses.append((end_x, end_y, end_yaw))
        return poses

    """Display"""
    def to_path_msg(self, stamp):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = stamp
        for x, y, yaw in self.points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = sin(yaw / 2.0)
            pose.pose.orientation.w = cos(yaw / 2.0)
            path.poses.append(pose)
        return path


def segment_distance(x, y, start, end):
    # Distance from (x, y) to the line segment start -> end
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    t = 0.0 if length_sq == 0.0 else max(0.0, min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy) / length_sq))
    return hypot(x - (start[0] + t * dx), y - (start[1] + t * dy))
