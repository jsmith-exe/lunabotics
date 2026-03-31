import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Pose2D
import tf2_ros
import tf_transformations


class AprilTagPose2D(Node):

    def __init__(self):
        super().__init__('apriltag_pose_2d')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Pose2D, '/apriltag_pose_2d', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'tag36h11:0',   # target (tag)
                'base_footprint',       # source (robot)
                rclpy.time.Time(),
                timeout = Duration(seconds=0.5)
            )

            # --- Translation ---
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # --- Rotation ---
            q = transform.transform.rotation
            quaternion = [q.x, q.y, q.z, q.w]

            roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

            theta = yaw

            # --- Publish ---
            pose = Pose2D()
            pose.x = x
            pose.y = y
            pose.theta = theta

            self.publisher.publish(pose)

        except Exception as e:
            self.get_logger().warn(f"TF error: {e}", throttle_duration_sec=2)
            return


def main():
    rclpy.init()
    node = AprilTagPose2D()
    rclpy.spin(node)
    rclpy.shutdown()