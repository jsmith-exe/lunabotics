import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros


class AprilTagGlobalPosePublisher(Node):

    def __init__(self):
        super().__init__('apriltag_global_pose_publisher')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/apriltag/pose',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        # Reject stale detections
        self.max_tag_age_sec = 0.3

        # Measurement covariance for robot_localization
        # 6x6 row-major: x, y, z, roll, pitch, yaw
        # These are starting values only and will need tuning.
        self.covariance = [
            0.03, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.03, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.03, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.05, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.05, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05,
        ]

    def is_transform_fresh(self, transform, max_age_sec=None):
        if max_age_sec is None:
            max_age_sec = self.max_tag_age_sec

        tf_time = Time.from_msg(transform.header.stamp)
        now = self.get_clock().now()
        age_sec = (now - tf_time).nanoseconds / 1e9
        return 0.0 <= age_sec < max_age_sec, age_sec

    def timer_callback(self):
        try:
            # Because the tag defines the map frame:
            # T^map_base = T^tag_base
            tag_T_base = self.tf_buffer.lookup_transform(
                'tag36h11:0',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )

            is_fresh, age_sec = self.is_transform_fresh(tag_T_base)
            if not is_fresh:
                self.get_logger().info(
                    f'AprilTag transform stale ({age_sec:.3f}s old), skipping publish',
                    throttle_duration_sec=2.0
                )
                return

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'

            # Full 3D translation
            msg.pose.pose.position.x = tag_T_base.transform.translation.x
            msg.pose.pose.position.y = tag_T_base.transform.translation.y
            msg.pose.pose.position.z = tag_T_base.transform.translation.z

            # Full 3D orientation
            msg.pose.pose.orientation = tag_T_base.transform.rotation

            msg.pose.covariance = self.covariance

            self.pose_pub.publish(msg)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().info(
                f'AprilTag transform unavailable: {e}',
                throttle_duration_sec=5.0
            )


def main():
    rclpy.init()
    node = AprilTagGlobalPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()