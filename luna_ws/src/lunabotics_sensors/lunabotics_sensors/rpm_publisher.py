import math

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from .base_publisher import BasePublisher


class RPMPublisher(BasePublisher):
    def __init__(self) -> None:
        # Rover parameters (must match xacro)
        self.wheel_radius = 0.12 / 2.0
        self.wheel_separation = 0.30

        self.last_linear_x = 0.0
        self.last_angular_z = 0.0

        super().__init__(
            node_name="rpm_pub",
            msg_type=Float32MultiArray,
            topic_name="/wheel_rpm",
            publish_rate_hz=10.0,
        )

        # Subscribe to odom
        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        # RViz marker publisher
        self.marker_pub = self.create_publisher(
            Marker, "/rpm_marker", 10
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.last_linear_x = msg.twist.twist.linear.x
        self.last_angular_z = msg.twist.twist.angular.z

    def build_message(self) -> Float32MultiArray:
        # Diff-drive kinematics
        v_left = (
            self.last_linear_x
            - self.last_angular_z * self.wheel_separation / 2.0
        )
        v_right = (
            self.last_linear_x
            + self.last_angular_z * self.wheel_separation / 2.0
        )

        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        rpm_left = omega_left * 60.0 / (2.0 * math.pi)
        rpm_right = omega_right * 60.0 / (2.0 * math.pi)

        msg = Float32MultiArray()
        msg.data = [rpm_left, rpm_right]

        self.publish_marker(rpm_left, rpm_right)

        return msg

    def publish_marker(self, rpm_left: float, rpm_right: float) -> None:
        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "rpm"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position = Point(x=0.0, y=-2.0, z=1.0)

        marker.scale.z = 0.6

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.text = (
            f"L: {rpm_left:.1f} RPM\n"
            f"R: {rpm_right:.1f} RPM"
        )

        self.marker_pub.publish(marker)


def main() -> None:
    RPMPublisher.run()


if __name__ == "__main__":
    main()
