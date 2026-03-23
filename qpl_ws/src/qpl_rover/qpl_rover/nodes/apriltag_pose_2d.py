#!/usr/bin/env python3
"""
apriltag_pose_2d.py

Subscribes to /detections and /back_camera/camera_info.
Computes rover (x, y, theta) with the apriltag as origin
using cv2.solvePnP on the detected corners.
Publishes Pose2D on /rover_pose_2d.

Tag coordinate frame (from solvePnP):
  X = along tag horizontal edge
  Y = along tag vertical edge
  Z = out of tag face (towards camera)

For a 2D floor map with tag as origin:
  arena X (forward from tag) = tag Z
  arena Y (lateral)          = tag X
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import math
import cv2

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose2D


class AprilTagPose2D(Node):
    def __init__(self):
        super().__init__('apriltag_pose_2d')

        self.declare_parameter('tag_size', 0.2)
        self.tag_size = self.get_parameter('tag_size').value

        # camera intrinsics — filled by camera_info callback
        self.camera_matrix = None
        self.dist_coeffs = None

        # 3D corners of the tag in tag frame (tag centred at origin, XY plane)
        half = self.tag_size / 2.0
        self.tag_corners_3d = np.array([
            [-half, -half, 0.0],
            [ half, -half, 0.0],
            [ half,  half, 0.0],
            [-half,  half, 0.0],
        ], dtype=np.float64)

        # tf2 for camera -> base_link from URDF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # pub / sub
        self.pose_pub = self.create_publisher(Pose2D, '/rover_pose_2d', 10)

        self.create_subscription(
            CameraInfo,
            '/back_camera/camera_info',
            self.camera_info_cb,
            10
        )

        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_cb,
            10
        )

        self.get_logger().info('apriltag_pose_2d started — waiting for camera_info...')

    def camera_info_cb(self, msg):
        """Grab camera intrinsics once."""
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info('Got camera intrinsics')

    def detection_cb(self, msg):
        if not msg.detections:
            return
        if self.camera_matrix is None:
            return

        det = msg.detections[0]
        cam_frame = msg.header.frame_id

        # ── step 1: get 2D corners from detection ────────────────
        corners_2d = np.array([
            [det.corners[0].x, det.corners[0].y],
            [det.corners[1].x, det.corners[1].y],
            [det.corners[2].x, det.corners[2].y],
            [det.corners[3].x, det.corners[3].y],
        ], dtype=np.float64)

        # ── step 2: solvePnP to get tag pose in camera frame ────
        success, rvec, tvec = cv2.solvePnP(
            self.tag_corners_3d,
            corners_2d,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if not success:
            self.get_logger().warn('solvePnP failed')
            return

        R_cam_tag, _ = cv2.Rodrigues(rvec)

        T_cam_tag = np.eye(4)
        T_cam_tag[:3, :3] = R_cam_tag
        T_cam_tag[:3, 3] = tvec.flatten()

        # ── step 3: get base_link in camera frame from URDF ─────
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                cam_frame,
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF lookup failed: {e}',
                throttle_duration_sec=2.0
            )
            return

        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        T_cam_base = self.quat_to_mat(t.x, t.y, t.z, r.x, r.y, r.z, r.w)

        # ── step 4: rover pose in tag frame ──────────────────────
        T_tag_base = np.linalg.inv(T_cam_tag) @ T_cam_base

        # ── step 5: remap to 2D floor coordinates ────────────────
        #
        # tag frame: X=right along tag, Y=down along tag, Z=out of face
        # floor map: arena_x = tag Z (forward from tag)
        #            arena_y = tag X (lateral)
        #
        # position
        arena_x = T_tag_base[2, 3]
        arena_y = T_tag_base[0, 3]

        # yaw: project rover's forward (base_link X axis) onto arena floor
        # first column of rotation = base_link X axis expressed in tag frame
        rover_fwd_arena_x = T_tag_base[2, 0]
        rover_fwd_arena_y = T_tag_base[0, 0]
        yaw = math.atan2(rover_fwd_arena_y, rover_fwd_arena_x)

        # publish
        pose = Pose2D()
        pose.x = float(arena_x)
        pose.y = float(arena_y)
        pose.theta = float(yaw)
        self.pose_pub.publish(pose)

        self.get_logger().info(
            f'x={arena_x:.3f}m  y={arena_y:.3f}m  theta={math.degrees(yaw):.1f}deg'
        )

    def quat_to_mat(self, px, py, pz, qx, qy, qz, qw):
        """Convert position + quaternion to 4x4 homogeneous matrix."""
        x2 = qx * qx
        y2 = qy * qy
        z2 = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        mat = np.eye(4)
        mat[:3, :3] = [
            [1 - 2*(y2+z2),  2*(xy-wz),      2*(xz+wy)],
            [2*(xy+wz),      1 - 2*(x2+z2),  2*(yz-wx)],
            [2*(xz-wy),      2*(yz+wx),      1 - 2*(x2+y2)]
        ]
        mat[:3, 3] = [px, py, pz]
        return mat


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPose2D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()