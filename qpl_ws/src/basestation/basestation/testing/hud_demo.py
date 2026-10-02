"""Fake rover for exercising the teleop HUD without the rover or Gazebo.

Publishes everything teleop_hud reads: TF (map -> odom -> base_footprint ->
camera optical frames), /odometry/filtered, /joint_states, the command topics,
drum/lift topics, camera_info plus synthetic JPEG camera frames (a projected
ground grid, so the HUD's drive guides can be checked against it), a costmap,
a plan, AprilTag fixes and some log lines. The rover follows a loop around the
arena, reverses now and then, hands over to "autonomy" for a stretch, and
briefly stalls a wheel, so every HUD state shows up within a couple of minutes.

    ros2 run basestation hud_demo
    ros2 launch basestation hud.launch.py transport:=compressed

It publishes on the real command topic names, so it refuses to start if
anything else is already on them (twist_mux, diff_cont, a live rover). Use a
private ROS_DOMAIN_ID anyway.
"""

import math
import random
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import JointState, CompressedImage, CameraInfo
from std_msgs.msg import Float64, Float64MultiArray


GUARDED_TOPICS = ["/cmd_vel_teleop", "/cmd_vel_nav", "/diff_cont/cmd_vel_unstamped",
                  "/drum_spin_control/teleop", "/drum_cont/commands", "/drum_lift_cont/commands"]

WAYPOINTS = [(3.4, 1.2), (1.6, 2.3), (5.6, 2.2), (7.0, 4.4), (8.1, 5.6), (6.4, 3.4), (4.2, 1.6)]
ROCKS = [(2.6, 4.6, 0.18), (5.0, 5.4, 0.22), (6.2, 1.4, 0.15), (4.3, 3.6, 0.2)]
MAP_T_ODOM = (0.25, 0.15, 0.06)  # x, y, yaw: a little drift between map and odom
WHEEL_R = 0.15
SEP = 0.5836
IMG_W, IMG_H = 960, 600
CYCLE = 90.0  # s: teleop for most of it, then a stretch of "autonomy"


def quat(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(x=sr * cp * cy - cr * sp * sy, y=cr * sp * cy + sr * cp * sy,
                      z=cr * cp * sy - sr * sp * cy, w=cr * cp * cy + sr * sp * sy)


def mat_to_quat(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return Quaternion(w=0.25 * s, x=(m[2][1] - m[1][2]) / s, y=(m[0][2] - m[2][0]) / s, z=(m[1][0] - m[0][1]) / s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return Quaternion(w=(m[2][1] - m[1][2]) / s, x=0.25 * s, y=(m[0][1] + m[1][0]) / s, z=(m[0][2] + m[2][0]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return Quaternion(w=(m[0][2] - m[2][0]) / s, x=(m[0][1] + m[1][0]) / s, y=0.25 * s, z=(m[1][2] + m[2][1]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return Quaternion(w=(m[1][0] - m[0][1]) / s, x=(m[0][2] + m[2][0]) / s, y=(m[1][2] + m[2][1]) / s, z=0.25 * s)


def optical_rotation(forward_sign, pitch_down):
    """Columns = optical x (right), y (down), z (forward) in base_footprint."""
    z = np.array([forward_sign * math.cos(pitch_down), 0.0, -math.sin(pitch_down)])
    x = np.array([0.0, -forward_sign, 0.0])
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


class Cam:
    def __init__(self, name, sign):
        self.name = name
        self.frame = f"{name}_optical_frame"
        self.R = optical_rotation(sign, math.radians(18))
        self.t = np.array([sign * 0.55, 0.0, 0.62])
        f = IMG_W / 2 / math.tan(math.radians(87) / 2)
        self.K = (f, f, IMG_W / 2, IMG_H / 2)


class HudDemo(Node):
    def __init__(self):
        super().__init__("hud_demo")
        self.pubs = {}

        def pub(mtype, topic, qos=10):
            self.pubs[topic] = self.create_publisher(mtype, topic, qos)

        for t in ["/cmd_vel_teleop", "/cmd_vel_nav", "/diff_cont/cmd_vel_unstamped"]:
            pub(Twist, t)
        pub(Odometry, "/odometry/filtered")
        pub(JointState, "/joint_states")
        pub(Float64, "/drum_spin_control/teleop")
        pub(Float64, "/drum_lift_control/teleop")
        pub(Float64MultiArray, "/drum_cont/commands")
        pub(Float64MultiArray, "/drum_lift_cont/commands")
        pub(PoseWithCovarianceStamped, "/apriltag/pose")
        pub(Path, "/plan")
        pub(OccupancyGrid, "/global_costmap/costmap",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.cams = [Cam("camera_front", 1), Cam("camera_rear", -1)]
        for c in self.cams:
            side = "front" if "front" in c.name else "rear"
            pub(CompressedImage, f"/depth_camera_{side}/color/image_raw/compressed", 2)
            pub(CameraInfo, f"/depth_camera_{side}/color/camera_info", 2)

        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform([self._static_cam_tf(c) for c in self.cams])

        self.x, self.y, self.yaw = WAYPOINTS[0][0], WAYPOINTS[0][1], math.pi / 2
        self.v = self.w = 0.0
        self.wp = 1
        self.t0 = time.time()
        self.drum_phase = 0.0
        self.lift = 0.8
        self.lift_target = 0.8
        self.reverse_until = 0.0
        self.next_reverse = self.t0 + 20.0
        self.costmap = self._make_costmap()

        self.create_timer(0.02, self.step)          # 50 Hz motion, odom, TF
        self.create_timer(1 / 15.0, self.cameras)   # 15 Hz camera frames
        self.create_timer(1.0, self.slow)
        self.create_timer(7.0, self.chatter)

    # ---------------------------------------------------------------- motion
    def phase(self, now):
        return "AUTO" if (now - self.t0) % CYCLE > CYCLE - 20 else "TELEOP"

    def step(self):
        now = time.time()
        dt = 0.02
        tx, ty = WAYPOINTS[self.wp]
        if math.hypot(tx - self.x, ty - self.y) < 0.35:
            self.wp = (self.wp + 1) % len(WAYPOINTS)
            tx, ty = WAYPOINTS[self.wp]
        err = math.atan2(ty - self.y, tx - self.x) - self.yaw
        err = math.atan2(math.sin(err), math.cos(err))
        cmd_v = 0.45 * max(0.0, math.cos(err))
        cmd_w = max(-0.8, min(0.8, 1.6 * err))
        if now > self.next_reverse:
            self.reverse_until = now + 3.0
            self.next_reverse = now + 30.0
        if now < self.reverse_until:
            cmd_v, cmd_w = -0.3, 0.15
        # First-order response so measured lags command, as on the real rover.
        self.v += (cmd_v - self.v) * min(1.0, dt * 4)
        self.w += (cmd_w - self.w) * min(1.0, dt * 5)
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = cmd_v, cmd_w
        src = "/cmd_vel_nav" if self.phase(now) == "AUTO" else "/cmd_vel_teleop"
        self.pubs[src].publish(cmd)
        self.pubs["/diff_cont/cmd_vel_unstamped"].publish(cmd)

        el = now - self.t0
        roll = math.radians(4 * math.sin(el / 3.1) + (9 if 50 < el % 120 < 56 else 0))
        pitch = math.radians(6 * math.sin(el / 4.3) + (8 if 52 < el % 120 < 57 else 0))
        stamp = self.get_clock().now().to_msg()

        # odom pose = inverse(map_T_odom) * map pose
        ox, oy, oyaw = MAP_T_ODOM
        dx, dy = self.x - ox, self.y - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        px, py, pyaw = c * dx - s * dy, s * dx + c * dy, self.yaw - oyaw

        t_mo = TransformStamped()
        t_mo.header.stamp = stamp
        t_mo.header.frame_id, t_mo.child_frame_id = "map", "odom"
        t_mo.transform.translation.x, t_mo.transform.translation.y = ox, oy
        t_mo.transform.rotation = quat(yaw=oyaw)
        t_ob = TransformStamped()
        t_ob.header.stamp = stamp
        t_ob.header.frame_id, t_ob.child_frame_id = "odom", "base_footprint"
        t_ob.transform.translation.x, t_ob.transform.translation.y = px, py
        t_ob.transform.rotation = quat(roll, pitch, pyaw)
        self.tf.sendTransform([t_mo, t_ob])

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id, od.child_frame_id = "odom", "base_footprint"
        od.pose.pose.position.x, od.pose.pose.position.y = px, py
        od.pose.pose.orientation = t_ob.transform.rotation
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w
        self.pubs["/odometry/filtered"].publish(od)

        wl = (self.v - self.w * SEP / 2) / WHEEL_R
        wr = (self.v + self.w * SEP / 2) / WHEEL_R
        stall = 70 < el % 120 < 73  # front-left wheel jams for a few seconds
        in_dig = self.y < 3.0 and self.phase(now) == "TELEOP"
        drum_cmd = 0.6 if in_dig else 0.0
        self.lift_target = 0.15 if in_dig else 0.85
        self.lift += max(-0.01, min(0.01, self.lift_target - self.lift))
        drum_vel = drum_cmd * 6.0
        self.drum_phase += drum_vel * dt
        js = JointState()
        js.header.stamp = stamp
        js.name = ["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint",
                   "rear_right_wheel_joint", "left_linear_actuator_joint", "right_linear_actuator_joint",
                   "drum_spin_joint"]
        js.position = [0.0] * 4 + [self.lift, self.lift + 0.01, self.drum_phase]
        js.velocity = [0.0 if stall else wl, wr, wl, wr, 0.0, 0.0, drum_vel]
        self.pubs["/joint_states"].publish(js)
        self.pubs["/drum_spin_control/teleop"].publish(Float64(data=drum_cmd))
        self.pubs["/drum_cont/commands"].publish(Float64MultiArray(data=[drum_cmd]))
        self.pubs["/drum_lift_cont/commands"].publish(Float64MultiArray(data=[self.lift_target] * 2))

    # --------------------------------------------------------------- cameras
    def _static_cam_tf(self, cam):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "base_footprint", cam.frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = cam.t.tolist()
        t.transform.rotation = mat_to_quat(cam.R.tolist())
        return t

    def _render(self, cam):
        img = np.zeros((IMG_H, IMG_W, 3), np.uint8)
        img[:] = (18, 26, 38)  # regolith brown-grey (BGR)
        fx, fy, cx, cy = cam.K
        c, s = math.cos(self.yaw), math.sin(self.yaw)

        def proj(wx, wy, wz=0.0):
            # world -> base -> optical
            dx, dy = wx - self.x, wy - self.y
            b = np.array([c * dx + s * dy, -s * dx + c * dy, wz])
            p = cam.R.T @ (b - cam.t)
            if p[2] < 0.1:
                return None
            return int(fx * p[0] / p[2] + cx), int(fy * p[1] / p[2] + cy)

        # Horizon/sky band.
        hz = proj(self.x + c * 200 * (1 if "front" in cam.name else -1), self.y + s * 200 * (1 if "front" in cam.name else -1))
        if hz:
            cv2.rectangle(img, (0, 0), (IMG_W, max(0, hz[1])), (40, 30, 24), -1)
        # World-fixed 0.5 m ground grid near the rover.
        gx0, gy0 = math.floor(self.x) - 5, math.floor(self.y) - 5
        for i in range(0, 21):
            for (ax, ay, bx, by) in ((gx0 + i * 0.5, gy0, gx0 + i * 0.5, gy0 + 10),
                                     (gx0, gy0 + i * 0.5, gx0 + 10, gy0 + i * 0.5)):
                pts = []
                for k in range(41):
                    q = proj(ax + (bx - ax) * k / 40, ay + (by - ay) * k / 40)
                    if q:
                        pts.append(q)
                if len(pts) > 1:
                    cv2.polylines(img, [np.array(pts, np.int32)], False, (60, 80, 100), 1, cv2.LINE_AA)
        for (rx, ry, rr) in ROCKS:
            ring = [proj(rx + rr * math.cos(a), ry + rr * math.sin(a), 0.05) for a in np.linspace(0, 2 * math.pi, 24)]
            ring = [q for q in ring if q]
            if len(ring) > 3:
                cv2.fillPoly(img, [np.array(ring, np.int32)], (70, 95, 120), cv2.LINE_AA)
        cv2.putText(img, f"SIMULATED {cam.name.split('_')[1].upper()} FEED  t={time.time() - self.t0:6.1f}s",
                    (16, IMG_H - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 150, 170), 1, cv2.LINE_AA)
        return img

    def cameras(self):
        stamp = self.get_clock().now().to_msg()
        for cam in self.cams:
            side = "front" if "front" in cam.name else "rear"
            ok, jpg = cv2.imencode(".jpg", self._render(cam), [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                continue
            m = CompressedImage()
            m.header.stamp = stamp
            m.header.frame_id = cam.frame
            m.format = "rgb8; jpeg compressed bgr8"
            m.data = jpg.tobytes()
            self.pubs[f"/depth_camera_{side}/color/image_raw/compressed"].publish(m)
            info = CameraInfo()
            info.header = m.header
            info.width, info.height = IMG_W, IMG_H
            fx, fy, cx, cy = cam.K
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            self.pubs[f"/depth_camera_{side}/color/camera_info"].publish(info)

    # ------------------------------------------------------------ slow stuff
    def _make_costmap(self):
        res, ox, oy = 0.05, -0.5, -0.5
        w, h = int(10.2 / res), int(9.1 / res)
        ys, xs = np.mgrid[0:h, 0:w]
        wx, wy = ox + (xs + 0.5) * res, oy + (ys + 0.5) * res
        cost = np.zeros((h, w), np.int16)
        for (rx, ry, rr) in ROCKS:
            d = np.hypot(wx - rx, wy - ry)
            infl = np.clip(98 * np.exp(-3.0 * np.maximum(0, d - rr)), 0, 98).astype(np.int16)
            infl[d > rr + 0.6] = 0
            cost = np.maximum(cost, infl)
            cost[d <= rr] = 100
        g = OccupancyGrid()
        g.header.frame_id = "map"
        g.info.resolution = res
        g.info.width, g.info.height = w, h
        g.info.origin.position.x, g.info.origin.position.y = ox, oy
        g.info.origin.orientation.w = 1.0
        g.data = cost.flatten().astype(np.int8).tolist()
        return g

    def slow(self):
        stamp = self.get_clock().now().to_msg()
        self.costmap.header.stamp = stamp
        self.pubs["/global_costmap/costmap"].publish(self.costmap)
        if self.phase(time.time()) == "AUTO":
            tx, ty = WAYPOINTS[self.wp]
            path = Path()
            path.header.frame_id = "map"
            path.header.stamp = stamp
            for k in range(31):
                f = k / 30
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = self.x + (tx - self.x) * f
                ps.pose.position.y = self.y + (ty - self.y) * f + 0.4 * math.sin(math.pi * f)
                path.poses.append(ps)
            self.pubs["/plan"].publish(path)
        if self.y < 4.5 and random.random() < 0.5:
            tag = PoseWithCovarianceStamped()
            tag.header.frame_id = "map"
            tag.header.stamp = stamp
            tag.pose.pose.position.x = self.x + random.gauss(0, 0.04)
            tag.pose.pose.position.y = self.y + random.gauss(0, 0.04)
            tag.pose.pose.orientation = quat(yaw=self.yaw)
            self.pubs["/apriltag/pose"].publish(tag)

    def chatter(self):
        # One call site per severity: rclpy refuses to change the severity of
        # a given call site between calls.
        log = self.get_logger()
        pick = random.randrange(4)
        if pick == 0:
            log.info("excavation cycle %d: drum at 0.60" % random.randint(1, 9))
        elif pick == 1:
            log.warn("front_left_wheel: CAN frame late (12 ms)")
        elif pick == 2:
            log.info("apriltag: tag_0 detected, 0 outliers rejected")
        else:
            log.error("drum_command_interface: lift actuator current limit hit")


def main(args=None):
    rclpy.init(args=args)
    probe = rclpy.create_node("hud_demo_probe")
    # Let discovery settle, then make sure nothing real is on the command topics.
    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)
    def others(infos):  # the HUD itself only listens, so it doesn't count
        return [i for i in infos if i.node_name != "teleop_hud"]

    busy = [t for t in GUARDED_TOPICS
            if others(probe.get_publishers_info_by_topic(t)) or others(probe.get_subscriptions_info_by_topic(t))]
    probe.destroy_node()
    if busy:
        print("hud_demo: refusing to start, these command topics are already in use: " + ", ".join(busy))
        print("hud_demo: is a rover or sim running on this ROS_DOMAIN_ID? Use a private one.")
        rclpy.shutdown()
        return
    node = HudDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
