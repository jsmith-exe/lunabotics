import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import tf_transformations
import numpy as np
from pupil_apriltags import Detector


class AprilTagObserver(Node):
    def __init__(self):
        super().__init__('apriltag_observer')

        # 1. INITIALIZE DATA STRUCTURES FIRST
        # This prevents the "AttributeError" if a callback triggers immediately
        self.bridge = CvBridge()
        self.cam_params = {'front': None, 'rear': None}
        self.tag_id = 0
        self.tag_size = 0.3  # Meters

        # 2. CONFIGURE DETECTOR
        # quad_decimate=2.0 and nthreads=4 to prevent EKF "Failed to meet update rate" errors
        self.detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25
        )

        # 3. CALCULATE STATIC TRANSFORMS
        # Tag pose in MAP frame (From your Gazebo world file)
        tag_pos = [0.055, 0.2, 0.3]
        tag_q = tf_transformations.quaternion_from_euler(1.5708, 0, 1.5708)
        self.T_map_tag = self.make_tf_matrix(tag_pos, tag_q)

        # Front Camera Optical relative to base_link
        f_pos = [0.25, 0.0, 0.04]
        f_q = tf_transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2)
        self.T_base_cam_front = self.make_tf_matrix(f_pos, f_q)

        # Rear Camera Optical relative to base_link
        r_pos = [-0.25, 0.0, 0.04]
        r_q_base_link = tf_transformations.quaternion_from_euler(0, 0, np.pi)
        r_q_link_opt = tf_transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2)
        r_q_combined = tf_transformations.quaternion_multiply(r_q_base_link, r_q_link_opt)
        self.T_base_cam_rear = self.make_tf_matrix(r_pos, r_q_combined)

        # 4. SETUP PUBLISHER
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/apriltag/pose', 10)

        # 5. CREATE SUBSCRIPTIONS LAST
        # Front Camera
        self.create_subscription(CameraInfo, '/depth_camera_front/camera_info',
                                 lambda msg: self.info_cb(msg, 'front'), 10)
        self.create_subscription(Image, '/depth_camera_front/image_raw',
                                 lambda msg: self.image_cb(msg, 'front'), 10)

        # Rear Camera
        self.create_subscription(CameraInfo, '/depth_camera_rear/camera_info',
                                 lambda msg: self.info_cb(msg, 'rear'), 10)
        self.create_subscription(Image, '/depth_camera_rear/image_raw',
                                 lambda msg: self.image_cb(msg, 'rear'), 10)

        self.get_logger().info("AprilTag Observer Initialized. Listening to Front & Rear.")

    def make_tf_matrix(self, pos, q):
        T = tf_transformations.quaternion_matrix(q)
        T[0:3, 3] = pos
        return T

    def info_cb(self, msg, cam_id):
        # Extract [fx, fy, cx, cy]
        self.cam_params[cam_id] = [msg.k[0], msg.k[4], msg.k[2], msg.k[5]]

    def image_cb(self, msg, cam_id):
        # Diagnostic: Check if we are even getting frames
        self.get_logger().info(f"IMAGE RECEIVED from {cam_id}", once=True)

        if self.cam_params[cam_id] is None:
            self.get_logger().warn(f"STUCK: Waiting for CameraInfo on {cam_id}...")
            return

        # 1. Convert to grayscale for detector
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        # 2. Run Detection
        results = self.detector.detect(cv_image,
                                       estimate_tag_pose=True,
                                       camera_params=self.cam_params[cam_id],
                                       tag_size=self.tag_size)

        if len(results) > 0:
            # Diagnostic: Tell us what IDs are actually being seen
            for r in results:
                self.get_logger().info(f"!!! DETECTED TAG ID: {r.tag_id} !!!")

                # --- THE MAGIC FORMULA ---
                T_cam_tag = np.eye(4)
                T_cam_tag[0:3, 0:3] = r.pose_R
                T_cam_tag[0:3, 3] = r.pose_t.flatten()

                # T_base_cam: Static offset
                T_base_cam = self.T_base_cam_front if cam_id == 'front' else self.T_base_cam_rear

                # T_map_base = T_map_tag * inv(T_cam_tag) * inv(T_base_cam)
                T_map_base = self.T_map_tag @ np.linalg.inv(T_cam_tag) @ np.linalg.inv(T_base_cam)

                # Publish regardless of ID for now to force the TF bridge to connect
                self.publish_pose(T_map_base, msg.header.stamp)
                self.get_logger().info(f"Published pose for Tag ID {r.tag_id}")

        else:
            # If this spams "sees nothing" even when looking at the tag,
            # then Gazebo's image is too blurry/low-res for the detector.
            self.get_logger().info("Detector sees nothing.", once=True)
            pass

    def publish_pose(self, T, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'

        pos = T[0:3, 3]
        q = tf_transformations.quaternion_from_matrix(T)

        msg.pose.pose.position.x = pos[0]
        msg.pose.pose.position.y = pos[1]
        msg.pose.pose.position.z = 0.0  # Force to ground plane for 2D nav

        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Covariance: Low values mean high trust.
        # [x, y, z, roll, pitch, yaw]
        msg.pose.covariance = np.diag([0.05, 0.05, 0.1, 0.1, 0.1, 0.05]).flatten().tolist()

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