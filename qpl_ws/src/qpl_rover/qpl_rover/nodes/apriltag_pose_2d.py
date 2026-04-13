import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Pose2D, TransformStamped
import tf2_ros
import tf_transformations
import math


class AprilTagMapOdom(Node):

    def __init__(self):
        super().__init__('apriltag_map_odom')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.publisher = self.create_publisher(Pose2D, '/apriltag_pose_2d', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.alpha = 0.3

        self.have_map_odom = False
        self.last_map_to_odom = None

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

    def timer_callback(self):
        try:
            # --- Tag -> base_footprint ---
            transform = self.tf_buffer.lookup_transform(
                'tag36h11:0',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )

            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # Same planar mapping you said you are happy with
            x_raw = tz
            y_raw = ty

            q = transform.transform.rotation
            quaternion = [q.x, q.y, q.z, q.w]
            rot = tf_transformations.quaternion_matrix(quaternion)

            # base +x axis expressed in tag frame
            fwd_x_tag = rot[0, 0]
            fwd_y_tag = rot[1, 0]
            fwd_z_tag = rot[2, 0]

            # Same projected 2D plane as translation mapping
            fwd_plane_x = fwd_z_tag
            fwd_plane_y = fwd_y_tag

            theta_raw = math.atan2(fwd_plane_y, fwd_plane_x)

            x_map = self.filter_value(self.prev_x, x_raw)
            y_map = self.filter_value(self.prev_y, y_raw)
            yaw_map = self.filter_angle(self.prev_theta, theta_raw)

            self.prev_x = x_map
            self.prev_y = y_map
            self.prev_theta = yaw_map

            pose = Pose2D()
            pose.x = x_map
            pose.y = y_map
            pose.theta = yaw_map
            self.publisher.publish(pose)

            # --- Odom -> base_footprint ---
            odom_tf = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )

            x_odom = odom_tf.transform.translation.x
            y_odom = odom_tf.transform.translation.y

            q_odom = odom_tf.transform.rotation
            _, _, yaw_odom = tf_transformations.euler_from_quaternion(
                [q_odom.x, q_odom.y, q_odom.z, q_odom.w]
            )
            yaw_odom = math.atan2(math.sin(yaw_odom), math.cos(yaw_odom))

            # --- Compute map -> odom ---
            yaw_mo = math.atan2(
                math.sin(yaw_map - yaw_odom),
                math.cos(yaw_map - yaw_odom)
            )

            x_mo = x_map - (math.cos(yaw_mo) * x_odom - math.sin(yaw_mo) * y_odom)
            y_mo = y_map - (math.sin(yaw_mo) * x_odom + math.cos(yaw_mo) * y_odom)

            q_mo = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw_mo)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = x_mo
            t.transform.translation.y = y_mo
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q_mo[0]
            t.transform.rotation.y = q_mo[1]
            t.transform.rotation.z = q_mo[2]
            t.transform.rotation.w = q_mo[3]

            self.last_map_to_odom = t
            self.have_map_odom = True
            self.tf_broadcaster.sendTransform(t)

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
    node = AprilTagMapOdom()
    rclpy.spin(node)
    rclpy.shutdown()