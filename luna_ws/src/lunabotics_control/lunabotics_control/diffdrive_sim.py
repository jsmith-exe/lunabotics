import math
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


@dataclass
class DiffDriveParams:
    wheel_radius_m: float = 0.06
    wheel_base_m: float = 0.30
    publish_rate_hz: float = 30.0

    # Frames
    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_link"

    # If True, publish a static transform map->odom (identity)
    publish_map_to_odom: bool = True

    # Joint names must match URDF joint names
    left_joints: List[str] = None
    right_joints: List[str] = None


class DiffDriveSim(Node):
    def __init__(self) -> None:
        super().__init__("diffdrive_sim")

        # ---- Parameters (can be overridden in launch) ----
        self.p = DiffDriveParams(
            wheel_radius_m=float(self.declare_parameter("wheel_radius_m", 0.06).value),
            wheel_base_m=float(self.declare_parameter("wheel_base_m", 0.30).value),
            publish_rate_hz=float(self.declare_parameter("publish_rate_hz", 30.0).value),
            map_frame=str(self.declare_parameter("map_frame", "map").value),
            odom_frame=str(self.declare_parameter("odom_frame", "odom").value),
            base_frame=str(self.declare_parameter("base_frame", "base_link").value),
            publish_map_to_odom=bool(self.declare_parameter("publish_map_to_odom", True).value),
        )

        # Default joint names (override if needed)
        self.p.left_joints = list(self.declare_parameter(
            "left_joints",
            ["front_left_wheel_joint", "rear_left_wheel_joint"]
        ).value)
        self.p.right_joints = list(self.declare_parameter(
            "right_joints",
            ["front_right_wheel_joint", "rear_right_wheel_joint"]
        ).value)

        # ---- State ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0  # linear m/s
        self.w = 0.0  # angular rad/s

        self.wheel_angle_left = 0.0
        self.wheel_angle_right = 0.0
        self.wheel_vel_left = 0.0
        self.wheel_vel_right = 0.0

        self.last_time = self.get_clock().now()

        # ---- ROS interfaces ----
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        
        qos_js = QoSProfile(depth=10)
        qos_js.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_js.durability = DurabilityPolicy.VOLATILE

        self.js_pub = self.create_publisher(JointState, "/joint_states", qos_js)


        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Optional static map->odom so RViz can use Fixed Frame = map
        if self.p.publish_map_to_odom:
            self.publish_static_map_to_odom()

        self.timer = self.create_timer(1.0 / self.p.publish_rate_hz, self.tick)

        self.get_logger().info(
            "diffdrive_sim: /cmd_vel -> publishes /odom, /joint_states, TF odom->base_link"
        )
        self.get_logger().info(
            f"Frames: map={self.p.map_frame} odom={self.p.odom_frame} base={self.p.base_frame} | "
            f"map->odom static={'ON' if self.p.publish_map_to_odom else 'OFF'}"
        )
        self.get_logger().info(
            f"Joints: left={self.p.left_joints} right={self.p.right_joints}"
        )

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        # ---- Integrate base pose (unicycle model) ----
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = self._wrap_pi(self.yaw + self.w * dt)

        # ---- Convert cmd_vel to wheel angular velocities ----
        v_l = self.v - (self.w * self.p.wheel_base_m / 2.0)  # m/s
        v_r = self.v + (self.w * self.p.wheel_base_m / 2.0)  # m/s

        self.wheel_vel_left = v_l / self.p.wheel_radius_m   # rad/s
        self.wheel_vel_right = v_r / self.p.wheel_radius_m  # rad/s

        # ---- Integrate wheel angles ----
        self.wheel_angle_left = self._wrap_2pi(self.wheel_angle_left + self.wheel_vel_left * dt)
        self.wheel_angle_right = self._wrap_2pi(self.wheel_angle_right + self.wheel_vel_right * dt)

        # ---- Publish outputs ----
        self.publish_tf(now)
        self.publish_odom(now)
        self.publish_joint_states(now)

    def publish_static_map_to_odom(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.p.map_frame
        t.child_frame_id = self.p.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

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
        names = self.p.left_joints + self.p.right_joints

        # positions per joint (left joints share angle, right joints share angle)
        positions = [self.wheel_angle_left] * len(self.p.left_joints) + \
                    [self.wheel_angle_right] * len(self.p.right_joints)

        velocities = [self.wheel_vel_left] * len(self.p.left_joints) + \
                     [self.wheel_vel_right] * len(self.p.right_joints)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = names
        msg.position = [float(p) for p in positions]
        msg.velocity = [float(v) for v in velocities]
        # effort left empty (sim)

        self.js_pub.publish(msg)

    @staticmethod
    def yaw_to_quat(yaw: float):
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    @staticmethod
    def _wrap_pi(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _wrap_2pi(a: float) -> float:
        a = a % (2.0 * math.pi)
        return a


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
