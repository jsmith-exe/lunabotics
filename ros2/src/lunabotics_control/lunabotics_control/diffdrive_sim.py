import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from tf2_ros import TransformBroadcaster


@dataclass
class DiffDriveParams:
    wheel_radius_m: float = 0.06     # must match your URDF wheel radius
    wheel_base_m: float = 0.30       # distance between left/right wheels
    publish_rate_hz: float = 30.0

    odom_frame: str = "odom"
    base_frame: str = "base_link"


class DiffDriveSim(Node):
    def __init__(self) -> None:
        super().__init__("diffdrive_sim")

        self.p = DiffDriveParams()

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0      # m/s
        self.w = 0.0      # rad/s

        # Wheel angles (for RViz animation)
        self.wheel_angle_left = 0.0
        self.wheel_angle_right = 0.0

        self.last_time = self.get_clock().now()

        # ROS interfaces
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / self.p.publish_rate_hz, self.tick)

        self.get_logger().info("diffdrive_sim running: subscribes /cmd_vel, publishes /odom, /joint_states, and TF odom->base_link")

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        # Integrate base pose (unicycle model)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # wrap to [-pi, pi]

        # Convert cmd_vel to wheel angular velocities
        v_l = self.v - (self.w * self.p.wheel_base_m / 2.0)   # m/s
        v_r = self.v + (self.w * self.p.wheel_base_m / 2.0)   # m/s
        w_l = v_l / self.p.wheel_radius_m                      # rad/s
        w_r = v_r / self.p.wheel_radius_m                      # rad/s

        # Integrate wheel angles
        self.wheel_angle_left += w_l * dt
        self.wheel_angle_right += w_r * dt

        # Publish TF odom -> base_link
        self.publish_tf(now)

        # Publish Odometry
        self.publish_odom(now)

        # Publish JointState (animate wheels)
        self.publish_joint_states(now)

    def publish_tf(self, now) -> None:
        qx, qy, qz, qw = self.yaw_to_quat(self.yaw)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.p.odom_frame
        t.child_frame_id = self.p.base_frame

        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def publish_odom(self, now) -> None:
        qx, qy, qz, qw = self.yaw_to_quat(self.yaw)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.p.odom_frame
        msg.child_frame_id = self.p.base_frame

        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = float(self.v)
        msg.twist.twist.angular.z = float(self.w)

        self.odom_pub.publish(msg)

    def publish_joint_states(self, now) -> None:
        # These joint names MUST match your URDF joint names
        names = [
            "front_left_wheel_joint",
            "rear_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_right_wheel_joint",
        ]

        # Left wheels same angle, right wheels same angle
        positions = [
            float(self.wheel_angle_left),
            float(self.wheel_angle_left),
            float(self.wheel_angle_right),
            float(self.wheel_angle_right),
        ]

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = names
        msg.position = positions

        self.js_pub.publish(msg)

    @staticmethod
    def yaw_to_quat(yaw: float):
        # roll=pitch=0
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))


def main() -> None:
    rclpy.init()
    node = DiffDriveSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
