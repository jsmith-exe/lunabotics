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

        self.pose_pub = self.create_publisher(Pose2D, '/apriltag_pose_2d', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.alpha = 0.2

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
        if not self.tf_buffer.can_transform(
            'tag36h11:0',
            'base_footprint',
            rclpy.time.Time(),
            timeout=Duration(seconds=0.05)
        ):
            self.publish_identity_or_last_map_odom()
            self.get_logger().info(
                'AprilTag not visible yet, holding map -> odom',
                throttle_duration_sec=5.0
            )
            return

        try:
            # Tag-based robot pose: tag frame is being used as map
            tag_tf = self.tf_buffer.lookup_transform(
                'tag36h11:0',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )

            # Axis mapping for your setup
            x_raw = tag_tf.transform.translation.z
            y_raw = tag_tf.transform.translation.y

            # Optional offsets if you want map origin shifted from tag origin
            x_map_raw = x_raw
            y_map_raw = y_raw

            q_tag = tag_tf.transform.rotation
            _, _, yaw_map_raw = tf_transformations.euler_from_quaternion(
                [q_tag.x, q_tag.y, q_tag.z, q_tag.w]
            )
            yaw_map_raw = math.atan2(math.sin(yaw_map_raw), math.cos(yaw_map_raw))

            x_map = self.filter_value(self.prev_x, x_map_raw)
            y_map = self.filter_value(self.prev_y, y_map_raw)
            yaw_map = self.filter_angle(self.prev_yaw, yaw_map_raw)

            self.prev_x = x_map
            self.prev_y = y_map
            self.prev_yaw = yaw_map

            pose = Pose2D()
            pose.x = x_map
            pose.y = y_map
            pose.theta = yaw_map
            self.pose_pub.publish(pose)

            # Odom pose of robot
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

            # map -> odom
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
            self.get_logger().warn(f"TF error: {e}", throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = AprilTagMapOdom()
    rclpy.spin(node)
    rclpy.shutdown()