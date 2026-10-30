#!/usr/bin/env python3
"""
Smoke test for the Gazebo Harmonic sand sim (sim.launch.py + gz_terramechanics).

Start the sim first, then run this against it:

  ros2 launch qpl_rover sim.launch.py headless:=true use_vslam:=false obstacle_seed:=1
  python3 qpl_ws/src/qpl_rover/test/sim/sand_sim_check.py

It drives through the normal teleop path (/cmd_vel_teleop -> drive_mux -> diff_cont),
reads the rover's true pose from Gazebo and each wheel's slip, load and sinkage from
the terramechanics plugin (/terra/<wheel>/state on Gazebo transport), and checks:

  1. the rover settles with all four wheels on the sand, carrying its weight
  2. driving forward moves it forward, with positive slip that stays sensible
  3. reversing moves it backward
  4. turning in place yaws it
  5. once stopped it stays put (no drift from the zero-friction contact, no NaN forces)

Timings are in sim time, so a slow machine (real time factor < 1) just takes longer.
Exits non-zero if any check fails.
"""

import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from gz.msgs10.float_v_pb2 import Float_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

WORLD = "lunabotics_us_arena_terrain"
MODEL = "rover"
WHEELS = ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]
SIM_ROVER_MASS = 25.3 + 4.0  # kg; the sim adds four 1 kg sand-spring links (rover_core.xacro)

# /terra/<wheel>/state layout (gz_terramechanics publishResults)
OMEGA, VX, VY, SLIP, SLIP_ANGLE, LOAD, SINKAGE = range(7)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def roll_pitch_of(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class SandSimCheck(Node):
    def __init__(self):
        super().__init__("sand_sim_check")
        self.lock = threading.Lock()
        self.sim_time = None
        self.pose = None  # (x, y, z, yaw, roll, pitch)
        self.wheel = {w: None for w in WHEELS}
        self.odom_seen = False
        self.failures = []

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(Odometry, "/diff_cont/odom", self._on_odom, 10)

        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f"/world/{WORLD}/pose/info", self._on_pose)
        for w in WHEELS:
            self.gz.subscribe(Float_V, f"/terra/{w}/state", self._wheel_cb(w))

    # ---------- callbacks ----------
    def _on_clock(self, msg):
        with self.lock:
            self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_odom(self, _msg):
        self.odom_seen = True

    def _on_pose(self, msg):
        for p in msg.pose:
            if p.name == MODEL:
                roll, pitch = roll_pitch_of(p.orientation)
                with self.lock:
                    self.pose = (p.position.x, p.position.y, p.position.z,
                                 yaw_of(p.orientation), roll, pitch)
                return

    def _wheel_cb(self, wheel):
        def cb(msg):
            with self.lock:
                self.wheel[wheel] = list(msg.data)
        return cb

    # ---------- helpers ----------
    def now(self):
        with self.lock:
            return self.sim_time

    def wait_sim(self, seconds, cmd=None, sample=None):
        """Spin for `seconds` of sim time, publishing cmd at 20 Hz (drive_mux times out at 0.25 s)."""
        start = self.now()
        while self.now() is None or self.now() - start < seconds:
            if cmd is not None:
                self.cmd_pub.publish(cmd)
            if sample is not None:
                sample()
            rclpy.spin_once(self, timeout_sec=0.05)

    def snapshot(self):
        with self.lock:
            return self.pose, {w: (list(v) if v else None) for w, v in self.wheel.items()}

    def check(self, ok, what):
        print(f"  [{'PASS' if ok else 'FAIL'}] {what}")
        if not ok:
            self.failures.append(what)

    def drive(self, label, vx, wz, seconds):
        """Drive with a constant command; returns (start pose, end pose, mean slip per wheel)."""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        start, _ = self.snapshot()
        slips = {w: [] for w in WHEELS}
        worst_tilt = [0.0]

        def sample():
            pose, wheels = self.snapshot()
            if pose:
                worst_tilt[0] = max(worst_tilt[0], abs(pose[4]), abs(pose[5]))
            for w, v in wheels.items():
                if v and abs(v[OMEGA]) > 0.1:
                    slips[w].append(v[SLIP])

        # skip the first second while the wheels spin up
        self.wait_sim(1.0, cmd)
        self.wait_sim(seconds - 1.0, cmd, sample)
        end, _ = self.snapshot()
        self.wait_sim(1.0, Twist())  # brake before the next manoeuvre
        mean_slip = {w: (sum(s) / len(s) if s else float("nan")) for w, s in slips.items()}
        print(f"\n{label}: cmd vx={vx} m/s wz={wz} rad/s for {seconds} s sim time")
        print(f"  start x={start[0]:.3f} y={start[1]:.3f} yaw={math.degrees(start[3]):.1f} deg")
        print(f"  end   x={end[0]:.3f} y={end[1]:.3f} yaw={math.degrees(end[3]):.1f} deg")
        print("  mean slip: " + ", ".join(f"{w}={s:+.3f}" for w, s in mean_slip.items()))
        print(f"  worst roll/pitch: {math.degrees(worst_tilt[0]):.1f} deg")
        self.check(worst_tilt[0] < math.radians(20), f"{label}: rover stays upright")
        return start, end, mean_slip

    # ---------- the test ----------
    def run(self):
        print("Waiting for the sim, controllers and terramechanics plugin...")
        deadline = time.time() + 300
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            pose, wheels = self.snapshot()
            if self.now() and pose and self.odom_seen and all(wheels.values()):
                break
        else:
            pose, wheels = self.snapshot()
            print(f"  clock={self.now()} pose={pose} odom={self.odom_seen} "
                  f"wheels={[w for w, v in wheels.items() if v]}")
            self.check(False, "sim, diff_cont and the plugin all came up within 300 s")
            return

        # 1. settle
        self.wait_sim(3.0)
        pose, wheels = self.snapshot()
        loads = {w: v[LOAD] for w, v in wheels.items()}
        sinkage = {w: v[SINKAGE] for w, v in wheels.items()}
        weight = SIM_ROVER_MASS * 9.81
        print("\nSettled on the sand:")
        print(f"  pose x={pose[0]:.3f} y={pose[1]:.3f} z={pose[2]:.3f} "
              f"roll={math.degrees(pose[4]):.1f} pitch={math.degrees(pose[5]):.1f} deg")
        print("  load [N]:    " + ", ".join(f"{w}={v:.1f}" for w, v in loads.items()))
        print("  sinkage [mm]: " + ", ".join(f"{w}={1000 * v:.1f}" for w, v in sinkage.items()))
        print(f"  total load {sum(loads.values()):.1f} N vs weight {weight:.1f} N")
        self.check(all(v > 0.1 * weight / 4 for v in loads.values()),
                   "all four wheels carry load")
        self.check(abs(sum(loads.values()) - weight) < 0.25 * weight,
                   "wheel loads add up to the rover's weight (within 25%)")
        self.check(all(0.0 < v < 0.05 for v in sinkage.values()),
                   "sinkage is positive and under 5 cm")

        # 2. forward
        start, end, slip = self.drive("Forward", 0.3, 0.0, 6.0)
        heading = start[3]
        forward = (end[0] - start[0]) * math.cos(heading) + (end[1] - start[1]) * math.sin(heading)
        commanded = 0.3 * 6.0
        mean_slip = sum(slip.values()) / len(slip)
        expected = commanded * (1 - mean_slip)
        print(f"  moved {forward:.3f} m forward, {commanded:.2f} m commanded, "
              f"{expected:.3f} m expected from the plugin's slip")
        self.check(forward > 0.5 * commanded, "forward: rover moves forward (> 50% of commanded)")
        self.check(forward < 1.05 * commanded, "forward: no more than commanded (no free sliding)")
        self.check(all(0.0 <= s < 0.5 for s in slip.values()),
                   "forward: every wheel slips a little (0 <= s < 0.5), as on sand")
        self.check(abs(forward - expected) < 0.1 * commanded,
                   "forward: distance matches commanded x (1 - slip), within 10%")

        # 3. reverse
        start, end, _ = self.drive("Reverse", -0.3, 0.0, 4.0)
        heading = start[3]
        moved = (end[0] - start[0]) * math.cos(heading) + (end[1] - start[1]) * math.sin(heading)
        print(f"  moved {moved:.3f} m along the heading")
        self.check(moved < -0.5 * 0.3 * 4.0, "reverse: rover moves backward (> 50% of commanded)")

        # 4. turn in place
        start, end, _ = self.drive("Turn left", 0.0, 0.5, 6.0)
        dyaw = math.atan2(math.sin(end[3] - start[3]), math.cos(end[3] - start[3]))
        print(f"  yawed {math.degrees(dyaw):.1f} deg, {math.degrees(0.5 * 6.0):.1f} deg commanded")
        self.check(dyaw > math.radians(20), "turn: rover yaws left")

        # 5. hold still
        self.wait_sim(2.0)
        start, _ = self.snapshot()
        self.wait_sim(5.0)
        end, wheels = self.snapshot()
        drift = math.hypot(end[0] - start[0], end[1] - start[1])
        print(f"\nStopped: drifted {1000 * drift:.1f} mm in 5 s")
        self.check(drift < 0.02, "stopped: drifts under 2 cm in 5 s")
        self.check(all(v and all(math.isfinite(x) for x in v) for v in wheels.values()),
                   "plugin state is finite on every wheel")


def main():
    rclpy.init()
    node = SandSimCheck()
    try:
        node.run()
    except Exception as e:  # a crash is a failure, not a pass
        node.failures.append(f"check script crashed: {e!r}")
        raise
    finally:
        node.cmd_pub.publish(Twist())
        print("\n" + ("ALL CHECKS PASSED" if not node.failures else
                      f"{len(node.failures)} CHECK(S) FAILED:\n  - " + "\n  - ".join(node.failures)))
        ok = not node.failures
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
