#!/usr/bin/env python3
"""
Dead-reckoning drift evaluation (sim only).

Scores the odom-frame estimate against Gazebo ground truth
(/gazebo/model_states, from the gazebo_ros_state plugin in arena_april.world).

WHY IT MEASURES RELATIVE MOTION, NOT ABSOLUTE POSE
    Dead reckoning has no absolute reference by definition: the odom frame's
    origin is wherever the filter happened to start, and asking "how far is the
    estimate from the true position in world coordinates" would just be
    measuring that arbitrary offset. So this latches a reference pose for every
    source AND for ground truth at the same instant, then compares displacement
    since that instant, expressed in each one's own start frame. No absolute
    initial position is needed or assumed.

WHY THERE IS NO APRILTAG HANDLING HERE
    This scores /odometry/filtered, the LOCAL EKF, whose inputs are wheel odom,
    the IMU, and (optionally) visual odometry. Tags only ever enter the GLOBAL
    filter via /apriltag/pose. The test launches map_localisation not at all,
    so the tag pipeline is absent rather than merely ignored.

    python3 tools/dr_eval.py --duration 70 --csv run.csv --json run.json
"""
import argparse
import csv
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

MODEL_NAME = "rover"

# label -> topic. Every one of these is an odom-frame dead-reckoning estimate.
SOURCES = {
    "fused":  "/odometry/filtered",   # local EKF output - the thing that matters
    "wheel":  "/diff_cont/odom",      # raw wheel odometry, the baseline
    "vo":     "/vo/odom",             # rgbd_odometry, absent when use_vslam:=false
}


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rel_to(ref, pose):
    """Displacement of `pose` from `ref`, expressed in ref's own frame."""
    rx, ry, ryaw = ref
    x, y, yaw = pose
    dx, dy = x - rx, y - ry
    c, s = math.cos(ryaw), math.sin(ryaw)
    return (c * dx + s * dy, -s * dx + c * dy, wrap(yaw - ryaw))


class Track:
    """Drift of one source against truth, both measured from a latched start."""

    def __init__(self, label):
        self.label = label
        self.ref = None
        self.n = 0
        self.sum_pos = 0.0
        self.max_pos = 0.0
        self.sum_yaw = 0.0
        self.max_yaw = 0.0
        self.final_pos = None
        self.final_yaw = None
        self.msgs = 0

    def add(self, pos_err, yaw_err):
        self.n += 1
        self.sum_pos += pos_err
        self.max_pos = max(self.max_pos, pos_err)
        self.sum_yaw += abs(yaw_err)
        self.max_yaw = max(self.max_yaw, abs(yaw_err))
        self.final_pos = pos_err
        self.final_yaw = yaw_err

    def result(self, path_len):
        if self.n == 0:
            return None
        pct = 100.0 * self.final_pos / path_len if path_len > 0.05 else None
        return {
            "samples": self.n,
            "msgs": self.msgs,
            "final_pos_err_m": self.final_pos,
            "final_pos_err_pct_of_path": pct,
            "mean_pos_err_m": self.sum_pos / self.n,
            "max_pos_err_m": self.max_pos,
            "final_yaw_err_deg": math.degrees(self.final_yaw),
            "mean_yaw_err_deg": math.degrees(self.sum_yaw / self.n),
            "max_yaw_err_deg": math.degrees(self.max_yaw),
        }


class DrEval(Node):
    def __init__(self, csv_path):
        super().__init__("dr_eval")
        self.truth = None
        self.truth_ref = None
        self.truth_msgs = 0
        self.path_len = 0.0
        self._last_truth_xy = None
        self.latched = False
        self.tracks = {k: Track(k) for k in SOURCES}
        self.rows = []
        self.csv_path = csv_path

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ModelStates, "/gazebo/model_states",
                                 self.truth_cb, best_effort)
        for label, topic in SOURCES.items():
            self.create_subscription(
                Odometry, topic,
                lambda msg, l=label: self.source_cb(msg, l), 10)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def truth_cb(self, msg):
        try:
            i = msg.name.index(MODEL_NAME)
        except ValueError:
            return
        p = msg.pose[i]
        self.truth = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.truth_msgs += 1
        # True distance travelled, for normalising drift into a percentage.
        if self._last_truth_xy is not None:
            self.path_len += math.hypot(self.truth[0] - self._last_truth_xy[0],
                                        self.truth[1] - self._last_truth_xy[1])
        self._last_truth_xy = (self.truth[0], self.truth[1])

    def latch(self):
        """Freeze the common t0. Everything after is measured from here."""
        if self.truth is None:
            return False
        self.truth_ref = self.truth
        self.path_len = 0.0
        self._last_truth_xy = (self.truth[0], self.truth[1])
        self.latched = True
        return True

    def source_cb(self, msg, label):
        tr = self.tracks[label]
        tr.msgs += 1
        if not self.latched or self.truth_ref is None:
            return
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        if tr.ref is None:
            tr.ref = pose
            return
        est = rel_to(tr.ref, pose)
        gt = rel_to(self.truth_ref, self.truth)
        pos_err = math.hypot(est[0] - gt[0], est[1] - gt[1])
        yaw_err = wrap(est[2] - gt[2])
        tr.add(pos_err, yaw_err)
        self.rows.append((self.now(), label, est[0], est[1], est[2],
                          gt[0], gt[1], gt[2], pos_err, yaw_err, self.path_len))

    def summary(self, meta):
        out = {**meta, "truth_path_len_m": self.path_len,
               "truth_msgs": self.truth_msgs, "sources": {}}
        print("\n===== DEAD-RECKONING DRIFT =====")
        print(f"ground-truth path length: {self.path_len:.3f} m "
              f"({self.truth_msgs} truth msgs)")
        if self.truth_msgs == 0:
            print("NO GROUND TRUTH - is the gazebo_ros_state plugin loaded?")
        for label in SOURCES:
            r = self.tracks[label].result(self.path_len)
            out["sources"][label] = r
            if r is None:
                print(f"  {label:6s}: no data ({self.tracks[label].msgs} msgs)")
                continue
            pct = f"{r['final_pos_err_pct_of_path']:.2f}%" \
                if r["final_pos_err_pct_of_path"] is not None else "n/a"
            print(f"  {label:6s}: final {r['final_pos_err_m']:.3f} m ({pct} of path)  "
                  f"mean {r['mean_pos_err_m']:.3f}  max {r['max_pos_err_m']:.3f}  "
                  f"final_yaw {r['final_yaw_err_deg']:+.2f} deg")
        if self.csv_path and self.rows:
            with open(self.csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "source", "est_x", "est_y", "est_yaw",
                            "truth_x", "truth_y", "truth_yaw",
                            "pos_err", "yaw_err", "path_len"])
                w.writerows(self.rows)
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=70.0,
                    help="sim-seconds to evaluate for")
    ap.add_argument("--csv", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    rclpy.init()
    node = DrEval(args.csv)
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

    # The sim clock has to be live before t0 means anything: latching while
    # now() still reads 0 makes the very first /clock message look like the
    # whole run has already elapsed, and the eval exits immediately.
    while rclpy.ok() and node.now() == 0.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    while rclpy.ok() and not node.latch():
        rclpy.spin_once(node, timeout_sec=0.2)
    t0 = node.now()
    node.get_logger().info(f"latched t0 at sim {t0:.2f}, truth ref {node.truth_ref}")

    try:
        while rclpy.ok() and node.now() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass

    out = node.summary({"label": args.label, "sim_seconds": node.now() - t0})
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
