#!/usr/bin/env python3
"""
Localisation evaluation harness (sim only).

Compares the fused map-frame pose (/odometry/global) and the raw AprilTag
observations (/apriltag/pose) against Gazebo ground truth (/gazebo/model_states,
published by the gazebo_ros_state plugin added to arena_april.world).

Ground truth arrives in Gazebo world coordinates; the map frame's origin sits at
the arena's SW corner (see WORLD_TO_MAP_* below), so world -> map is a fixed
translation.

Run for a fixed duration, then print a summary:

    python3 tools/loc_eval.py --duration 60 --csv /tmp/loc_eval.csv
"""
import argparse
import csv
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates

# Arena corner (map origin) offset from the Gazebo world origin.
# UCF arena (arena_ucf.world): map = world + (4.57, 4.05).
# (Old UK arena_april.world was (2.2, 3.95).)
WORLD_TO_MAP_X = 4.57
WORLD_TO_MAP_Y = 4.05

MODEL_NAME = "rover"


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class Stats:
    def __init__(self):
        self.n = 0
        self.sum_pos = 0.0
        self.sum_yaw = 0.0
        self.max_pos = 0.0
        self.max_yaw = 0.0

    def add(self, pos_err, yaw_err):
        self.n += 1
        self.sum_pos += pos_err
        self.sum_yaw += abs(yaw_err)
        self.max_pos = max(self.max_pos, pos_err)
        self.max_yaw = max(self.max_yaw, abs(yaw_err))

    def report(self, label):
        if self.n == 0:
            return f"{label}: no samples"
        return (f"{label}: n={self.n}  mean_pos={self.sum_pos/self.n:.3f} m  "
                f"max_pos={self.max_pos:.3f} m  mean_yaw={math.degrees(self.sum_yaw/self.n):.2f} deg  "
                f"max_yaw={math.degrees(self.max_yaw):.2f} deg")


class LocEval(Node):
    def __init__(self, csv_path=None):
        super().__init__("loc_eval")
        self.truth = None  # (x, y, yaw) in map frame
        self.global_stats = Stats()
        self.tag_stats = Stats()
        self.global_count = 0
        self.tag_count = 0
        self.rows = []
        self.csv_path = csv_path

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ModelStates, "/gazebo/model_states", self.truth_cb, best_effort)
        self.create_subscription(Odometry, "/odometry/global", self.global_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/apriltag/pose", self.tag_cb, 10)
        self.create_timer(5.0, self.progress)

    def truth_cb(self, msg):
        try:
            i = msg.name.index(MODEL_NAME)
        except ValueError:
            return
        p = msg.pose[i]
        self.truth = (
            p.position.x + WORLD_TO_MAP_X,
            p.position.y + WORLD_TO_MAP_Y,
            yaw_from_quat(p.orientation),
        )

    def _record(self, source, x, y, yaw, stats):
        if self.truth is None:
            return
        tx, ty, tyaw = self.truth
        pos_err = math.hypot(x - tx, y - ty)
        yaw_err = ang_diff(yaw, tyaw)
        stats.add(pos_err, yaw_err)
        t = self.get_clock().now().nanoseconds / 1e9
        self.rows.append((t, source, x, y, yaw, tx, ty, tyaw, pos_err, yaw_err))

    def global_cb(self, msg):
        self.global_count += 1
        p = msg.pose.pose
        self._record("global", p.position.x, p.position.y,
                     yaw_from_quat(p.orientation), self.global_stats)

    def tag_cb(self, msg):
        self.tag_count += 1
        p = msg.pose.pose
        self._record("tag", p.position.x, p.position.y,
                     yaw_from_quat(p.orientation), self.tag_stats)

    def progress(self):
        truth = "ok" if self.truth is not None else "MISSING"
        self.get_logger().info(
            f"truth={truth} global_msgs={self.global_count} tag_msgs={self.tag_count}"
        )

    def summary(self):
        print("\n===== LOCALISATION EVAL SUMMARY =====")
        if self.truth is None:
            print("NO GROUND TRUTH RECEIVED — is the gazebo_ros_state plugin loaded?")
        print(self.global_stats.report("/odometry/global vs truth"))
        print(self.tag_stats.report("/apriltag/pose    vs truth"))
        print(f"messages: global={self.global_count} tag={self.tag_count}")
        if self.csv_path and self.rows:
            with open(self.csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "source", "x", "y", "yaw",
                            "truth_x", "truth_y", "truth_yaw", "pos_err", "yaw_err"])
                w.writerows(self.rows)
            print(f"wrote {len(self.rows)} rows to {self.csv_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    rclpy.init()
    node = LocEval(csv_path=args.csv)
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

    import time
    end = time.monotonic() + args.duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.summary()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
