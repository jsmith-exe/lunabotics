"""Teleop HUD: a browser-based replacement for the RViz teleop layouts.

Monitoring only. This node subscribes and never publishes a command, so the
driving controller (basestation main.py -> nav_pub) stays the one and only
source of teleop commands. It gathers what the operator needs into a handful
of JSON channels and serves them, with the camera feeds, to a local web page:

  config    arena, zones, rover footprint, camera intrinsics/extrinsics
  tele      20 Hz: pose (map + odom), attitude, commanded vs measured motion,
            wheel/drum/lift joints, control source, topic health
  costmap   global costmap as a PNG, at most once a second
  plan      latest Nav2 global plan
  log       recent /rosout lines

Open http://localhost:8765 (see launch/hud.launch.py).
"""

import base64
import math
import os
import time
from collections import deque

import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

import tf2_ros
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import JointState, CompressedImage, CameraInfo
from std_msgs.msg import Float64, Float64MultiArray
from rcl_interfaces.msg import Log

from basestation.hud.server import Hub, CameraFeed, HudServer


WHEEL_JOINTS = {
    "fl": "front_left_wheel_joint",
    "fr": "front_right_wheel_joint",
    "rl": "rear_left_wheel_joint",
    "rr": "rear_right_wheel_joint",
}
LIFT_JOINTS = {"l": "left_linear_actuator_joint", "r": "right_linear_actuator_joint"}
DRUM_JOINT = "drum_spin_joint"

# From qpl_rover/description/rover_core.xacro and config/my_controllers.yaml.
ROVER = {
    "length": 1.1746,   # base_long_len
    "width": 0.7436,    # base_short_len
    "wheel_radius": 0.15,
    "wheel_width": 0.15,
    "wheel_separation": 0.5836,
    "max_linear": 1.0,
    "max_angular": 1.0,
}

# A command source counts as live for this long after its last message. Twice
# the 0.25 s timeouts in config/drive_mux.yaml: nav_pub republishes at 5 Hz, so
# a 0.25 s window would make the CONTROL badge flicker on ordinary jitter.
SOURCE_HOLD = 0.5
TELE_HZ = 20.0
LOG_KEEP = 300
LOG_SEND = 120


def num(x, nd=4):
    """Round for the wire; NaN/inf become None so the JSON stays valid."""
    if x is None or not math.isfinite(x):
        return None
    return round(float(x), nd)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rpy_of(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    s = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2, s) if abs(s) >= 1 else math.asin(s)
    return roll, pitch, yaw_of(q)


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def jpeg_size(data):
    """(width, height) from a JPEG's SOF header, without decoding it."""
    i, n = 2, len(data)
    while i + 9 < n:
        if data[i] != 0xFF:
            return None
        marker = data[i + 1]
        seg = (data[i + 2] << 8) | data[i + 3]
        if marker in (0xC0, 0xC1, 0xC2):
            h = (data[i + 5] << 8) | data[i + 6]
            w = (data[i + 7] << 8) | data[i + 8]
            return w, h
        i += 2 + seg
    return None


class TopicStats:
    """Message rate and age for one subscription, for the DATA LINKS panel."""

    def __init__(self, name, kind):
        self.name = name
        self.kind = kind
        self.stamps = deque(maxlen=400)
        self.last = None
        self.bytes = deque(maxlen=400)

    def tick(self, now, size=0):
        self.last = now
        self.stamps.append(now)
        self.bytes.append((now, size))

    def summary(self, now):
        while self.stamps and now - self.stamps[0] > 2.0:
            self.stamps.popleft()
        while self.bytes and now - self.bytes[0][0] > 2.0:
            self.bytes.popleft()
        hz = len(self.stamps) / 2.0
        bps = sum(b for _, b in self.bytes) / 2.0
        age = None if self.last is None else now - self.last
        return {"name": self.name, "kind": self.kind, "hz": num(hz, 1),
                "age": num(age, 2), "bps": int(bps)}


class Camera:
    def __init__(self, key, label, topic):
        self.key = key
        self.label = label
        self.topic = topic
        self.feed = CameraFeed(key)
        self.stats = TopicStats(topic, "camera")
        self.size = None
        self.model = None  # intrinsics + extrinsics once known
        self.info = None
        self.info_sub = None


class HudNode(Node):
    def __init__(self):
        super().__init__("teleop_hud")

        p = self.declare_parameter
        self.port = p("port", 8765).value
        self.host = p("host", "127.0.0.1").value
        self.map_frame = p("map_frame", "map").value
        self.odom_frame = p("odom_frame", "odom").value
        self.base_frame = p("base_frame", "base_footprint").value
        front_topic = p("front_camera_topic", "/depth_camera_front/color/image_raw/compressed").value
        rear_topic = p("rear_camera_topic", "/depth_camera_rear/color/image_raw/compressed").value
        front_info = p("front_camera_info", "/depth_camera_front/color/camera_info").value
        rear_info = p("rear_camera_info", "/depth_camera_rear/color/camera_info").value
        web_root = p("web_root", "").value or os.path.join(
            get_package_share_directory("basestation"), "hud")

        self.hub = Hub()
        self.stats = {}
        self.state = {
            "odom": None,         # /odometry/filtered
            "cmd_teleop": None,   # (Twist, wall time)
            "cmd_nav": None,
            "cmd_out": None,
            "drum_teleop": None,  # (value, wall time)
            "lift_teleop": None,
            "drum_auto": None,
            "lift_auto": None,
            "drum_cmd": None,     # /drum_cont/commands
            "lift_cmd": None,     # /drum_lift_cont/commands
            "joints": {},         # name -> (pos, vel)
            "tag": None,          # (x, y, yaw, wall time)
        }
        self.logs = deque(maxlen=LOG_KEEP)
        self.log_id = 0
        self.log_dirty = False
        self.costmap_msg = None
        self.costmap_dirty = False
        self.costmap_sent = 0.0
        self.start_wall = time.time()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Best effort for the high-rate streams: over the Wi-Fi link a dropped
        # sample is better than a retransmit queue, and it still matches the
        # rover's reliable publishers.
        fast = qos_profile_sensor_data
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)

        self._sub(Odometry, "/odometry/filtered", self.on_odom, fast, "odom")
        self._sub(JointState, "/joint_states", self.on_joints, fast, "joints")
        self._sub(Twist, "/cmd_vel_teleop", lambda m: self._stamp("cmd_teleop", m), fast, "cmd")
        self._sub(Twist, "/cmd_vel_nav", lambda m: self._stamp("cmd_nav", m), fast, "cmd")
        self._sub(Twist, "/diff_cont/cmd_vel_unstamped", lambda m: self._stamp("cmd_out", m), fast, "cmd")
        self._sub(Float64, "/drum_spin_control/teleop", lambda m: self._stamp("drum_teleop", m.data), fast, "drum")
        self._sub(Float64, "/drum_lift_control/teleop", lambda m: self._stamp("lift_teleop", m.data), fast, "drum")
        self._sub(Float64, "/drum_spin_control/autonomy", lambda m: self._stamp("drum_auto", m.data), fast, "drum")
        self._sub(Float64, "/drum_lift_control/autonomy", lambda m: self._stamp("lift_auto", m.data), fast, "drum")
        self._sub(Float64MultiArray, "/drum_cont/commands",
                  lambda m: self._stamp("drum_cmd", list(m.data)), fast, "drum")
        self._sub(Float64MultiArray, "/drum_lift_cont/commands",
                  lambda m: self._stamp("lift_cmd", list(m.data)), fast, "drum")
        self._sub(PoseWithCovarianceStamped, "/apriltag/pose", self.on_tag, 10, "loc")
        self._sub(OccupancyGrid, "/global_costmap/costmap", self.on_costmap, latched, "nav")
        self._sub(Path, "/plan", self.on_plan, 10, "nav")
        self._sub(Log, "/rosout", self.on_log, QoSProfile(depth=200), "log")

        self.cameras = {
            "front": Camera("front", "FRONT", front_topic),
            "rear": Camera("rear", "REAR", rear_topic),
        }
        for cam, info in ((self.cameras["front"], front_info), (self.cameras["rear"], rear_info)):
            self.create_subscription(CompressedImage, cam.topic,
                                     lambda m, c=cam: self.on_image(c, m), fast)
            # Intrinsics are only needed once; the subscription is dropped after.
            cam.info_sub = self.create_subscription(
                CameraInfo, info, lambda m, c=cam: self.on_info(c, m), fast)

        self.arena = self._load_arena()
        self.config_dirty = True

        self.create_timer(1.0 / TELE_HZ, self.publish_tele)
        self.create_timer(0.25, self.publish_slow)
        self.create_timer(1.0, self.resolve_camera_models)

        self.server = HudServer(self.host, self.port, web_root, self.hub,
                                {k: c.feed for k, c in self.cameras.items()},
                                self.get_logger())
        self.server.start()
        shown = "localhost" if self.host in ("0.0.0.0", "127.0.0.1") else self.host
        self.get_logger().info(f"Teleop HUD serving {web_root} on http://{shown}:{self.port}")

    # ------------------------------------------------------------------ setup
    def _sub(self, mtype, topic, cb, qos, kind):
        stats = self.stats[topic] = TopicStats(topic, kind)

        def wrapped(msg):
            stats.tick(time.time())
            cb(msg)

        self.create_subscription(mtype, topic, wrapped, qos)

    def _stamp(self, key, value):
        self.state[key] = (value, time.time())

    def _load_arena(self):
        try:
            cfg_dir = os.path.join(get_package_share_directory("qpl_rover"), "config", "arena")
            with open(os.path.join(cfg_dir, "selector.yaml")) as f:
                name = yaml.safe_load(f)["arena"]
            with open(os.path.join(cfg_dir, f"{name}.yaml")) as f:
                arena = yaml.safe_load(f)["arena"]
            tag = arena.get("apriltag", {})
            return {
                "name": str(name).upper(),
                "width": arena["width"],
                "length": arena["length"],
                "buffer": arena.get("buffer", 0.0),
                "zones": arena.get("zones", []),
                "tag": tag.get("position"),
            }
        except Exception as e:  # the HUD is still useful without the arena
            self.get_logger().warn(f"Could not load arena config: {e}")
            return None

    # -------------------------------------------------------------- callbacks
    def on_odom(self, msg):
        self.state["odom"] = msg

    def on_joints(self, msg):
        joints = self.state["joints"]
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else None
            vel = msg.velocity[i] if i < len(msg.velocity) else None
            joints[name] = (pos, vel)

    def on_tag(self, msg):
        p = msg.pose.pose
        self.state["tag"] = (p.position.x, p.position.y, yaw_of(p.orientation), time.time(),
                             msg.header.frame_id)

    def on_costmap(self, msg):
        self.costmap_msg = msg
        self.costmap_dirty = True

    def on_plan(self, msg):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if len(pts) > 200:
            step = len(pts) / 200.0
            pts = [pts[int(i * step)] for i in range(200)] + [pts[-1]]
        self.hub.publish("plan", {"frame": msg.header.frame_id,
                                  "pts": [[num(x, 3), num(y, 3)] for x, y in pts]})

    def on_log(self, msg):
        if msg.name == self.get_name():
            return
        self.log_id += 1
        self.logs.append({"id": self.log_id, "t": msg.stamp.sec + msg.stamp.nanosec * 1e-9,
                          "lvl": int(msg.level), "node": msg.name, "msg": msg.msg[:400]})
        self.log_dirty = True

    def on_image(self, cam, msg):
        data = bytes(msg.data)
        fmt = msg.format.lower()
        if "png" in fmt:
            # Browsers would take PNG in MJPEG too, but re-encoding keeps frames small.
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return
            ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                return
            data = enc.tobytes()
        cam.stats.tick(time.time(), len(msg.data))
        if cam.size is None:
            cam.size = jpeg_size(data)
        cam.feed.push(data)

    def on_info(self, cam, msg):
        if msg.k[0] <= 0:
            return
        cam.info = msg
        if cam.info_sub is not None:
            self.destroy_subscription(cam.info_sub)
            cam.info_sub = None

    def resolve_camera_models(self):
        """Pair each camera's intrinsics with its pose on the rover, so the page
        can draw drive guides on the image. Static, so done once."""
        for cam in self.cameras.values():
            if cam.model is not None or cam.info is None:
                continue
            frame = cam.info.header.frame_id
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
            except tf2_ros.TransformException:
                continue
            tr = t.transform.translation
            k = cam.info.k
            cam.model = {
                "fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5],
                "w": cam.info.width, "h": cam.info.height,
                # Optical frame pose in base_footprint: rotation matrix (rows) + origin.
                "R": quat_to_mat(t.transform.rotation),
                "t": [tr.x, tr.y, tr.z],
                "frame": frame,
            }
            self.config_dirty = True
            self.get_logger().info(f"{cam.label} camera model resolved from {frame}")

    # ------------------------------------------------------------- publishers
    def _lookup(self, target):
        """Pose of the base in `target`, with its age, or None."""
        try:
            t = self.tf_buffer.lookup_transform(target, self.base_frame, Time())
        except tf2_ros.TransformException:
            return None
        stamp = Time.from_msg(t.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        tr, q = t.transform.translation, t.transform.rotation
        roll, pitch, yaw = rpy_of(q)
        return {"x": num(tr.x, 3), "y": num(tr.y, 3), "z": num(tr.z, 3),
                "roll": num(roll), "pitch": num(pitch), "yaw": num(yaw), "age": num(age, 2)}

    def _twist(self, entry, now):
        if entry is None:
            return None
        msg, t = entry
        return {"vx": num(msg.linear.x, 3), "wz": num(msg.angular.z, 3), "age": num(now - t, 2)}

    def _scalar(self, entry, now):
        if entry is None:
            return None
        v, t = entry
        if isinstance(v, list):
            v = v[0] if v else None
        return {"v": num(v, 3), "age": num(now - t, 2)}

    def publish_tele(self):
        now = time.time()
        s = self.state

        cmd_teleop = self._twist(s["cmd_teleop"], now)
        cmd_nav = self._twist(s["cmd_nav"], now)
        if cmd_teleop and cmd_teleop["age"] < SOURCE_HOLD:
            source = "TELEOP"
        elif cmd_nav and cmd_nav["age"] < SOURCE_HOLD:
            source = "AUTO"
        else:
            source = "IDLE"

        odom = None
        if s["odom"] is not None:
            tw = s["odom"].twist.twist
            odom = {"vx": num(tw.linear.x, 3), "vy": num(tw.linear.y, 3), "wz": num(tw.angular.z, 3),
                    "age": num(now - self.stats["/odometry/filtered"].last, 2)}

        joints = s["joints"]
        wheels = {k: num((joints.get(j) or (None, None))[1], 3) for k, j in WHEEL_JOINTS.items()}
        lift = {k: num((joints.get(j) or (None, None))[0], 3) for k, j in LIFT_JOINTS.items()}
        drum = joints.get(DRUM_JOINT)

        tag = None
        if s["tag"] is not None:
            x, y, yaw, t, frame = s["tag"]
            tag = {"x": num(x, 3), "y": num(y, 3), "yaw": num(yaw), "age": num(now - t, 1), "frame": frame}

        topics = [st.summary(now) for st in self.stats.values()]
        topics += [c.stats.summary(now) for c in self.cameras.values()]
        cams = {}
        for k, c in self.cameras.items():
            summ = c.stats.summary(now)
            cams[k] = {"hz": summ["hz"], "age": summ["age"], "bps": summ["bps"],
                       "size": list(c.size) if c.size else None}

        tele = {
            "t": round(now, 3),
            "up": round(now - self.start_wall, 1),
            "map": self._lookup(self.map_frame),
            "odom_pose": self._lookup(self.odom_frame),
            "odom": odom,
            "source": source,
            "cmd": {"teleop": cmd_teleop, "nav": cmd_nav, "out": self._twist(s["cmd_out"], now)},
            "wheels": wheels,
            "drum": {
                "vel": num(drum[1], 3) if drum else None,
                "teleop": self._scalar(s["drum_teleop"], now),
                "auto": self._scalar(s["drum_auto"], now),
                "cmd": self._scalar(s["drum_cmd"], now),
            },
            "lift": {
                "pos": lift,
                "teleop": self._scalar(s["lift_teleop"], now),
                "auto": self._scalar(s["lift_auto"], now),
                "cmd": self._scalar(s["lift_cmd"], now),
            },
            "tag": tag,
            "cams": cams,
            "topics": topics,
        }
        try:
            self.hub.publish("tele", tele)
        except ValueError as e:
            self.get_logger().warn(f"Dropped a telemetry frame: {e}", throttle_duration_sec=5.0)

    def publish_slow(self):
        if self.config_dirty:
            self.config_dirty = False
            self.hub.publish("config", {
                "arena": self.arena,
                "rover": ROVER,
                "frames": {"map": self.map_frame, "odom": self.odom_frame, "base": self.base_frame},
                "cams": {k: {"label": c.label, "topic": c.topic, "model": c.model}
                         for k, c in self.cameras.items()},
                "source_hold": SOURCE_HOLD,
            })
        if self.log_dirty:
            self.log_dirty = False
            self.hub.publish("log", list(self.logs)[-LOG_SEND:])
        if self.costmap_dirty and time.monotonic() - self.costmap_sent >= 1.0:
            self.costmap_dirty = False
            self.costmap_sent = time.monotonic()
            self.hub.publish("costmap", self._encode_costmap(self.costmap_msg))

    def _encode_costmap(self, msg):
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        rgba = np.zeros((info.height, info.width, 4), np.uint8)
        # Inflation: amber, alpha ramps with cost. Lethal/inscribed: red.
        infl = (grid > 0) & (grid < 99)
        rgba[infl] = (255, 140, 0, 0)
        rgba[..., 3][infl] = (40 + grid[infl] * 1.4).astype(np.uint8)
        lethal = grid >= 99
        rgba[lethal] = (255, 40, 40, 220)
        # Row 0 of the grid sits at origin.y; flip so the PNG reads top = +y.
        bgra = cv2.cvtColor(np.flipud(rgba), cv2.COLOR_RGBA2BGRA)
        ok, png = cv2.imencode(".png", bgra)
        o = info.origin
        return {
            "frame": msg.header.frame_id,
            "res": info.resolution, "w": info.width, "h": info.height,
            "ox": o.position.x, "oy": o.position.y, "oyaw": yaw_of(o.orientation),
            "png": base64.b64encode(png.tobytes()).decode() if ok else None,
        }

    def destroy_node(self):
        self.server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HudNode()
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
