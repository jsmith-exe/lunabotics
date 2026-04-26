import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import Pose2D, TransformStamped
import tf2_ros
import tf_transformations


class AprilTagMapOdom3D(Node):

    def __init__(self):
        super().__init__('apriltag_map_odom_3d')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.publisher = self.create_publisher(Pose2D, '/apriltag_pose_2d', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.alpha = 0.2

        self.have_map_odom = False
        self.last_map_to_odom = None

        self.max_tag_age_sec = 0.3

        # --------------------------------------------------------------
        # FIXED ROTATIONAL CORRECTION: map = correction * tag
        # --------------------------------------------------------------
        self.map_from_tag_roll = np.pi / 2
        self.map_from_tag_pitch = 0.0
        self.map_from_tag_yaw = np.pi / 2

        # --------------------------------------------------------------
        # If True, dynamically shift map->tag in Z so that
        # base_footprint always lies on z=0 in map frame.
        # --------------------------------------------------------------
        self.force_base_z_to_zero = True

        # --------------------------------------------------------------
        # Tag frames published by each detector. Tried in order;
        # first fresh one wins.
        # --------------------------------------------------------------
        self.tag_frames = ['tag36h11_front_0', 'tag36h11_rear_0']

    def filter_value(self, prev, new):
        if prev is None:
            return new
        return self.alpha * new + (1.0 - self.alpha) * prev

    def filter_angle(self, prev, new):
        if prev is None:
            return new
        diff = math.atan2(math.sin(new - prev), math.cos(new - prev))
        filtered = prev + self.alpha * diff
        return math.atan2(math.sin(filtered), math.cos(filtered))

    def is_transform_fresh(self, transform, max_age_sec=None):
        if max_age_sec is None:
            max_age_sec = self.max_tag_age_sec

        tf_time = Time.from_msg(transform.header.stamp)
        now = self.get_clock().now()
        age_sec = (now - tf_time).nanoseconds / 1e9

        return 0.0 <= age_sec < max_age_sec, age_sec

    def lookup_freshest_tag(self):
        for frame in self.tag_frames:
            try:
                tf = self.tf_buffer.lookup_transform(
                    frame,
                    'base_footprint',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
                is_fresh, _ = self.is_transform_fresh(tf)
                if is_fresh:
                    return tf, frame
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                continue
        return None, None

    def publish_identity_or_last_map_odom(self):
        now = self.get_clock().now().to_msg()

        if self.have_map_odom and self.last_map_to_odom is not None:
            self.last_map_to_odom.header.stamp = now
            self.tf_broadcaster.sendTransform(self.last_map_to_odom)
            return

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def transform_to_matrix(self, transform_msg):
        q = transform_msg.rotation
        t = transform_msg.translation

        quaternion = [q.x, q.y, q.z, q.w]
        T = tf_transformations.quaternion_matrix(quaternion)
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z
        return T

    def matrix_to_transform_stamped(self, T, parent_frame, child_frame):
        q = tf_transformations.quaternion_from_matrix(T)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame

        msg.transform.translation.x = float(T[0, 3])
        msg.transform.translation.y = float(T[1, 3])
        msg.transform.translation.z = float(T[2, 3])

        msg.transform.rotation.x = float(q[0])
        msg.transform.rotation.y = float(q[1])
        msg.transform.rotation.z = float(q[2])
        msg.transform.rotation.w = float(q[3])

        return msg

    def make_rpy_transform(self, roll_rad, pitch_rad, yaw_rad):
        q = tf_transformations.quaternion_from_euler(
            roll_rad, pitch_rad, yaw_rad
        )
        T = tf_transformations.quaternion_matrix(q)
        return T

    def make_translation_transform(self, x, y, z):
        T = np.eye(4)
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    def timer_callback(self):
        try:
            # --------------------------------------------------------------
            # 1) Lookup full 3D tag -> base transform from whichever
            #    detector has a fresh reading (front or rear).
            # --------------------------------------------------------------
            tag_T_base_msg, source_frame = self.lookup_freshest_tag()

            if tag_T_base_msg is None:
                self.publish_identity_or_last_map_odom()
                self.get_logger().info(
                    'No fresh AprilTag from any camera; holding last map->odom',
                    throttle_duration_sec=2.0,
                )
                return

            T_tag_base = self.transform_to_matrix(tag_T_base_msg.transform)

            # --------------------------------------------------------------
            # 2) Build rotational map -> tag correction
            # --------------------------------------------------------------
            T_map_tag_rot = self.make_rpy_transform(
                self.map_from_tag_roll,
                self.map_from_tag_pitch,
                self.map_from_tag_yaw
            )

            # First compute map->base using rotation only
            T_map_base_rot_only = T_map_tag_rot @ T_tag_base

            # --------------------------------------------------------------
            # 3) Dynamically compensate for AprilTag height
            #
            # We force base_footprint to sit on z = 0 in map frame.
            # So if current base z in map is +h, shift map->tag by -h in z.
            # --------------------------------------------------------------
            if self.force_base_z_to_zero:
                base_z_in_map = float(T_map_base_rot_only[2, 3])
                T_map_tag_z_fix = self.make_translation_transform(
                    0.0, 0.0, -base_z_in_map
                )
                T_map_tag = T_map_tag_z_fix @ T_map_tag_rot
            else:
                T_map_tag = T_map_tag_rot

            T_map_base = T_map_tag @ T_tag_base

            # --------------------------------------------------------------
            # 4) Lookup odom -> base
            # --------------------------------------------------------------
            odom_T_base_msg = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )

            T_odom_base = self.transform_to_matrix(odom_T_base_msg.transform)

            # --------------------------------------------------------------
            # 5) Compute map -> odom
            # --------------------------------------------------------------
            T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

            t_map_odom = self.matrix_to_transform_stamped(T_map_odom, 'map', 'odom')
            self.last_map_to_odom = t_map_odom
            self.have_map_odom = True
            self.tf_broadcaster.sendTransform(t_map_odom)

            # --------------------------------------------------------------
            # 6) Optional debug Pose2D
            # --------------------------------------------------------------
            x_raw = float(T_map_base[0, 3])
            y_raw = float(T_map_base[1, 3])
            yaw_raw = math.atan2(T_map_base[1, 0], T_map_base[0, 0])

            x_map = self.filter_value(self.prev_x, x_raw)
            y_map = self.filter_value(self.prev_y, y_raw)
            yaw_map = self.filter_angle(self.prev_theta, yaw_raw)

            self.prev_x = x_map
            self.prev_y = y_map
            self.prev_theta = yaw_map

            pose = Pose2D()
            pose.x = x_map
            pose.y = y_map
            pose.theta = yaw_map
            self.publisher.publish(pose)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.publish_identity_or_last_map_odom()
            self.get_logger().info(
                f'AprilTag transform not available yet: {e}',
                throttle_duration_sec=5.0
            )


def main():
    rclpy.init()
    node = AprilTagMapOdom3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()