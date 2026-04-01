import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Pose2D
import tf2_ros
import tf_transformations
import math


class AprilTagPose2D(Node):

    def __init__(self):
        super().__init__('apriltag_pose_2d')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Pose2D, '/apriltag_pose_2d', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        # --- Filtering ---
        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None

        self.alpha = 0.2  # smoothing factor (0 = no update, 1 = no filtering)

    def filter(self, prev, new):
        if prev is None:
            return new
        return self.alpha * new + (1 - self.alpha) * prev

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'tag36h11:0',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )

            # --- Translation (USE X & Y ONLY) ---
            x_raw = transform.transform.translation.x
            y_raw = transform.transform.translation.y

            # --- Rotation ---
            q = transform.transform.rotation
            quaternion = [q.x, q.y, q.z, q.w]

            roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

            theta_raw = yaw

            # --- Normalize angle (-pi to pi) ---
            theta_raw = math.atan2(math.sin(theta_raw), math.cos(theta_raw))

            # --- Apply filtering ---
            x = self.filter(self.prev_x, x_raw)
            y = self.filter(self.prev_y, y_raw)
            theta = self.filter(self.prev_theta, theta_raw)

            self.prev_x = x
            self.prev_y = y
            self.prev_theta = theta

            # --- Publish ---
            pose = Pose2D()
            pose.x = x
            pose.y = y
            pose.theta = theta

            self.publisher.publish(pose)

        except Exception as e:
            self.get_logger().warn(f"TF error: {e}", throttle_duration_sec=2)


def main():
    rclpy.init()
    node = AprilTagPose2D()
    rclpy.spin(node)
    rclpy.shutdown()