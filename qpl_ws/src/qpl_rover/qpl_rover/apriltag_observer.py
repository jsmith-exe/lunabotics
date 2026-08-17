import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import tf_transformations
import numpy as np
from pupil_apriltags import Detector

import tf2_ros


# THE ARENA GEOFENCE WITH BUFFER
# Physical Arena (UCF championship, arena_ucf.world): X(0 to 9.14), Y(0 to 8.10)
buffer = 0.20  # 20cm buffer to allow for jitter and edge-driving

x_min, x_max = 0.0 - buffer, 9.14 + buffer
y_min, y_max = 0.0 - buffer, 8.10 + buffer
z_limit = 0.50  # Z is usually the noisiest; give it a little more room


class AprilTagObserver(Node):
    def __init__(self):
        super().__init__('apriltag_observer')

        # 1. INITIALIZE DATA STRUCTURES FIRST
        # This prevents the "AttributeError" if a callback triggers immediately
        self.bridge = CvBridge()
        self.cam_params = {'front': None, 'rear': None}

        # --- Parameters -----------------------------------------------------
        # tag_map_pose is THE single source of truth for the map frame: the
        # global EKF fuses the poses computed against it, so wherever this says
        # the tag is, that is where the map frame ends up. map_localisation
        # launch passes the same values here and to the map->tag_0 static TF so
        # the two can never disagree.
        # Defaults match config/tag_pose.yaml (UCF arena: 1.5 m tag board
        # filling the fiducial rail on the front wall, centre at
        # map (3.32, 0.01, 0.75), facing +Y; yaw is the sanitised tag frame's
        # x-axis, which points INTO the wall).
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('tag_size', 1.2)  # between edges of the BLACK OUTLINE
        self.declare_parameter('tag_map_pose', [3.32, 0.01, 0.75, -np.pi / 2])  # x, y, z, yaw
        self.declare_parameter('decision_margin_min', 35.0)
        # Measurement noise at 1 m tag distance; scaled by distance^2 because
        # pose-from-homography error grows roughly quadratically with range.
        self.declare_parameter('cov_xy_at_1m', 0.02)
        self.declare_parameter('cov_yaw_at_1m', 0.05)
        # Range gate: beyond this the tag spans too few pixels — position is
        # noisy and yaw suffers the classic planar-tag ambiguity flip, which
        # rotates the whole map->odom transform and destabilises Nav2. Reject
        # rather than fuse-with-big-covariance. Scaled for the 1.2 m tag; see
        # tag_pose.yaml.
        self.declare_parameter('max_tag_range', 11.0)
        # Cooldown: minimum time between published poses (both cameras share
        # it). Every accepted pose makes the EKF rewind/replay and nudges
        # map->odom, so at full frame rate the TF flutters under Nav2 while
        # the tag is in view. A few absolute fixes per second is plenty to
        # bound drift.
        self.declare_parameter('min_update_period', 0.5)

        self.tag_id = self.get_parameter('tag_id').value
        self.tag_size = self.get_parameter('tag_size').value
        self.decision_margin_min = self.get_parameter('decision_margin_min').value
        self.cov_xy_at_1m = self.get_parameter('cov_xy_at_1m').value
        self.cov_yaw_at_1m = self.get_parameter('cov_yaw_at_1m').value
        self.max_tag_range = self.get_parameter('max_tag_range').value
        self.min_update_period = self.get_parameter('min_update_period').value
        self._last_pub_time = None

        # 2. CONFIGURE DETECTOR
        # nthreads=4 to prevent EKF "Failed to meet update rate" errors
        self.detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25
        )

        # 3. CALCULATE STATIC TRANSFORMS
        # Tag pose in MAP frame (from the parameter above)
        tx, ty, tz, tyaw = self.get_parameter('tag_map_pose').value
        tag_q = tf_transformations.quaternion_from_euler(0, 0, tyaw)
        self.T_map_tag = self.make_tf_matrix([tx, ty, tz], tag_q)

        # Camera extrinsics (base_footprint -> camera body frame) come from TF,
        # not hardcoded values: the URDF is the single source of truth and the
        # old hardcoded z=0.02 was missing the base_footprint->base_link offset
        # (0.1625 m). Resolved lazily on first use so we don't block startup.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_frames = {'front': 'camera_link_front', 'rear': 'camera_link_rear'}
        self.T_footprint_cam = {'front': None, 'rear': None}

        # 4. SETUP PUBLISHER
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/apriltag/pose', 10)

        # 5. CREATE SUBSCRIPTIONS LAST
        # Sensor-data QoS: real camera drivers publish best-effort; a reliable
        # subscription would silently receive nothing from them.
        # Front Camera
        self.create_subscription(CameraInfo, '/depth_camera_front/color/camera_info',
                                 lambda msg: self.info_cb(msg, 'front'), qos_profile_sensor_data)
        self.create_subscription(Image, '/depth_camera_front/color/image_raw',
                                 lambda msg: self.image_cb(msg, 'front'), qos_profile_sensor_data)

        # Rear Camera
        self.create_subscription(CameraInfo, '/depth_camera_rear/color/camera_info',
                                 lambda msg: self.info_cb(msg, 'rear'), qos_profile_sensor_data)
        self.create_subscription(Image, '/depth_camera_rear/color/image_raw',
                                 lambda msg: self.image_cb(msg, 'rear'), qos_profile_sensor_data)

        self.get_logger().info(
            f"AprilTag Observer Initialized. Tag {self.tag_id} at map "
            f"({tx:.3f}, {ty:.3f}, {tz:.3f}, yaw {tyaw:.3f}). Listening to Front & Rear."
        )

    def make_tf_matrix(self, pos, q):
        T = tf_transformations.quaternion_matrix(q)
        T[0:3, 3] = pos
        return T

    def info_cb(self, msg, cam_id):
        # Extract [fx, fy, cx, cy]
        self.cam_params[cam_id] = [msg.k[0], msg.k[4], msg.k[2], msg.k[5]]

    def get_footprint_cam_tf(self, cam_id):
        """base_footprint -> camera body frame from TF, cached once resolved."""
        if self.T_footprint_cam[cam_id] is not None:
            return self.T_footprint_cam[cam_id]
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_footprint', self.camera_frames[cam_id], rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.get_logger().warn(
                f"Waiting for TF base_footprint -> {self.camera_frames[cam_id]}",
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        self.T_footprint_cam[cam_id] = self.make_tf_matrix(
            [t.x, t.y, t.z], [r.x, r.y, r.z, r.w])
        self.get_logger().info(
            f"Resolved {cam_id} camera extrinsics: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
        return self.T_footprint_cam[cam_id]

    def sanitize_optical_to_ros(self, raw_r, raw_t):
        # ROS X (Forward) is Optical Z
        # ROS Y (Left) is -Optical X
        # ROS Z (Up) is -Optical Y
        ros_t = np.array([raw_t[2], -raw_t[0], -raw_t[1]])

        # Change of Basis Matrix
        S = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ])
        ros_r = S @ raw_r @ S.T
        return ros_r, ros_t

    def image_cb(self, msg, cam_id):
        if self.cam_params[cam_id] is None:
            self.get_logger().warn(f"STUCK: Waiting for CameraInfo on {cam_id}...",
                                   throttle_duration_sec=5.0)
            return

        try:
            # 1. Convert to grayscale for detector
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            # 2. Run Detection
            results = self.detector.detect(cv_image,
                                        estimate_tag_pose=True,
                                        camera_params=self.cam_params[cam_id],
                                        tag_size=self.tag_size)

            self.process_results(results, msg, cam_id)
        except Exception as e:
            self.get_logger().error(f"Exception during image callback: {e}")

    def process_results(self, results, msg, cam_id):
        if len(results) == 0:
            return
        T_footprint_cam = self.get_footprint_cam_tf(cam_id)
        if T_footprint_cam is None:
            return
        for r in results:
            # Only the tag whose map pose we know defines the map frame; any
            # other tag processed against that pose produces a wild ghost.
            if r.tag_id != self.tag_id:
                continue

            # Filter out low-quality detections or high ambiguity
            # decision_margin: how clear the tag is (higher is better, < 30 is risky)
            if r.decision_margin < self.decision_margin_min:
                self.get_logger().warn(f"Ignoring noisy detection (Margin: {r.decision_margin:.1f})")
                continue

            # Get raw data from detector
            raw_t = r.pose_t.flatten()
            raw_R = r.pose_R

            # SANITIZE: Convert Optical -> ROS Standard
            ros_R, ros_t = self.sanitize_optical_to_ros(raw_R, raw_t)

            # Range gate — see the max_tag_range parameter comment.
            tag_distance = float(np.linalg.norm(ros_t))
            if tag_distance > self.max_tag_range:
                self.get_logger().info(
                    f"Ignoring distant detection ({tag_distance:.1f} m > {self.max_tag_range:.1f} m)",
                    throttle_duration_sec=5.0)
                continue

            # Now create the CLEAN measurement matrix (Camera Body -> Tag)
            T_cam_tag = np.eye(4)
            T_cam_tag[0:3, 0:3] = ros_R
            T_cam_tag[0:3, 3] = ros_t

            # Calculate Map -> Tag -> Camera -> Footprint
            T_map_footprint = self.T_map_tag @ np.linalg.inv(T_cam_tag) @ np.linalg.inv(T_footprint_cam)

            # Extract Position
            pos = T_map_footprint[0:3, 3]

            # Boundary Check
            in_x = x_min <= pos[0] <= x_max
            in_y = y_min <= pos[1] <= y_max
            in_z = abs(pos[2]) <= z_limit

            if not (in_x and in_y and in_z):
                self.get_logger().warn(f"Ghost Rejected at X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f}; in_x={in_x}, in_y={in_y}, in_z={in_z}")
                continue

            # If we passed the fence, publish to EKF (rate-limited by the
            # min_update_period cooldown, shared across both cameras).
            now = self.get_clock().now()
            if (self._last_pub_time is not None
                    and (now - self._last_pub_time).nanoseconds / 1e9 < self.min_update_period):
                continue
            self._last_pub_time = now
            self.publish_pose(T_map_footprint, msg.header.stamp, tag_distance)

    def publish_pose(self, T, stamp, tag_distance):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'

        pos = T[0:3, 3]
        q = tf_transformations.quaternion_from_matrix(T)

        msg.pose.pose.position.x = pos[0]
        msg.pose.pose.position.y = pos[1]
        msg.pose.pose.position.z = pos[2]

        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Distance-scaled measurement noise. The old constant 0.001 (sigma
        # ~3 cm) claimed the same confidence at 7 m as at 1 m, which both let
        # noisy long-range detections yank the EKF and made the Mahalanobis
        # gate reject honest measurements after any drift.
        # Format: [x, y, z, roll, pitch, yaw]
        scale = max(1.0, tag_distance) ** 2
        cov_xy = self.cov_xy_at_1m * scale
        cov_yaw = self.cov_yaw_at_1m * scale
        diag = [cov_xy, cov_xy, cov_xy * 5.0, 0.5, 0.5, cov_yaw]
        msg.pose.covariance = np.diag(diag).flatten().tolist()

        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = AprilTagObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
