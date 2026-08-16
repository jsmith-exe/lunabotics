import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import tf_transformations
import numpy as np
from pupil_apriltags import Detector
import tf2_ros
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class AprilTagObserver(Node):
    def __init__(self):
        super().__init__('apriltag_observer')

        # Load arena configuration
        config_path = os.path.join(
            get_package_share_directory('qpl_rover'),
            'config',
            'arena',
            'uk.yaml'
        )

        with open(config_path, 'r') as file:
            arena_config = yaml.safe_load(file)['arena']

        self.arena_width = arena_config['width']
        self.arena_length = arena_config['length']
        self.buffer = arena_config['buffer']
        self.z_limit = arena_config['z_limit']

        self.x_min = -self.buffer
        self.x_max = self.arena_width + self.buffer
        self.y_min = -self.buffer
        self.y_max = self.arena_length + self.buffer

        # Load AprilTag configuration
        apriltag_config_path = os.path.join(
            get_package_share_directory('qpl_rover'),
            'config',
            'localisation',
            'apriltag.yaml'
        )

        with open(apriltag_config_path, 'r') as file:
            apriltag_config = yaml.safe_load(file)['apriltag']

        self.apriltag_physical_size = apriltag_config['physical_size']
        self.min_decision_margin = apriltag_config['min_decision_margin']

        # Physical tag size includes the white border, as used by Gazebo.
        # pupil_apriltags expects the black-to-black tag size, excluding the
        # 1-pixel white border and therefore uses 8/10 of the physical size.
        self.tag_size = self.apriltag_physical_size * 8 / 10

        # 1. INITIALIZE DATA STRUCTURES FIRST
        # This prevents the "AttributeError" if a callback triggers immediately
        self.bridge = CvBridge()
        self.cam_params = {'front': None, 'rear': None}

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
        # Tag pose in MAP frame
        tag_pos = arena_config['apriltag']['position']
        tag_q = arena_config['apriltag']['quaternion']
        self.T_map_tag = self.make_tf_matrix(tag_pos, tag_q)

        # TF listener for rover/camera transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 4. SETUP PUBLISHER
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/apriltag/pose', 10)

        # 5. CREATE SUBSCRIPTIONS LAST
        # Front Camera
        self.create_subscription(CameraInfo, '/depth_camera_front/color/camera_info',
                                 lambda msg: self.info_cb(msg, 'front'), 10)
        self.create_subscription(Image, '/depth_camera_front/color/image_raw',
                                 lambda msg: self.image_cb(msg, 'front'), 10)

        # Rear Camera
        self.create_subscription(CameraInfo, '/depth_camera_rear/color/camera_info',
                                 lambda msg: self.info_cb(msg, 'rear'), 10)
        self.create_subscription(Image, '/depth_camera_rear/color/image_raw',
                                 lambda msg: self.image_cb(msg, 'rear'), 10)

        self.get_logger().info("AprilTag Observer Initialized. Listening to Front & Rear.")

    def make_tf_matrix(self, pos, q):
        T = tf_transformations.quaternion_matrix(q)
        T[0:3, 3] = pos
        return T

    def info_cb(self, msg, cam_id):
        # Extract [fx, fy, cx, cy]
        self.cam_params[cam_id] = [msg.k[0], msg.k[4], msg.k[2], msg.k[5]]

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
        # Diagnostic: Check if we are even getting frames
        #self.get_logger().info(f"IMAGE RECEIVED from {cam_id}", once=True)

        if self.cam_params[cam_id] is None:
            self.get_logger().warn(f"STUCK: Waiting for CameraInfo on {cam_id}...")
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

    def get_camera_transform(self, cam_id):
        camera_frame = f'camera_link_{cam_id}'

        transform = self.tf_buffer.lookup_transform(
            'base_link',
            camera_frame,
            rclpy.time.Time()
        )

        t = transform.transform.translation
        q = transform.transform.rotation

        return self.make_tf_matrix(
            [t.x, t.y, t.z],
            [q.x, q.y, q.z, q.w]
        )
    
    def process_results(self, results, msg, cam_id):
        if len(results) == 0:
            # If this spams "sees nothing" even when looking at the tag,
            # then Gazebo's image is too blurry/low-res for the detector.
            #self.get_logger().info(f"{cam_id} detector sees nothing.", once=True)
            return
        for r in results:
            # Filter out low-quality detections or high ambiguity
            if r.decision_margin < self.min_decision_margin:
                self.get_logger().warn(f"Ignoring noisy detection (Margin: {r.decision_margin:.1f})")
                continue

            #self.get_logger().info(f"{cam_id} camera detected tag.")

            # Get raw data from detector
            raw_t = r.pose_t.flatten()
            raw_R = r.pose_R

            # SANITIZE: Convert Optical -> ROS Standard
            ros_R, ros_t = self.sanitize_optical_to_ros(raw_R, raw_t)

            # Now create the CLEAN measurement matrix (Camera Body -> Tag)
            T_cam_tag = np.eye(4)
            T_cam_tag[0:3, 0:3] = ros_R
            T_cam_tag[0:3, 3] = ros_t

            # Select the correct static transform relative to footprint
            T_footprint_cam = self.get_camera_transform(cam_id)

            # Calculate Map -> Tag -> Camera -> Footprint
            T_map_footprint = self.T_map_tag @ np.linalg.inv(T_cam_tag) @ np.linalg.inv(T_footprint_cam)

            # Extract Position (This is now the Ground Projection)
            pos = T_map_footprint[0:3, 3]

            # Boundary Check
            in_x = self.x_min <= pos[0] <= self.x_max
            in_y = self.y_min <= pos[1] <= self.y_max
            in_z = abs(pos[2]) <= self.z_limit

            if not (in_x and in_y and in_z):
                self.get_logger().warn(f"Ghost Rejected at X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f}; in_x={in_x}, in_y={in_y}, in_z={in_z}")
                continue

            # If we passed the fence, publish to EKF
            #self.get_logger().info(f"Position X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f}") # debug test message DO NOT REMOVE
            self.publish_pose(T_map_footprint, msg.header.stamp)

    def publish_pose(self, T, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'

        pos = T[0:3, 3]
        q = tf_transformations.quaternion_from_matrix(T)

        # 1. USE FULL 3D COORDINATES
        msg.pose.pose.position.x = pos[0]
        msg.pose.pose.position.y = pos[1]
        msg.pose.pose.position.z = pos[2]

        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # 2. ADJUST COVARIANCE FOR 3D
        # Now that we trust Z, we lower its covariance value.
        # Format: [x, y, z, roll, pitch, yaw]
        # We use 0.001 for high trust, and maybe 0.05 for rotation/Z
        diag = [0.001, 0.001, 0.005, 0.1, 0.1, 0.01]
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