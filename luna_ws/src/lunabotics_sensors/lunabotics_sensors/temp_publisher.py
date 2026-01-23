import random

from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from .base_publisher import BasePublisher


class TemperaturePublisher(BasePublisher):
    def __init__(self) -> None:
        super().__init__(
            node_name="temp_pub",
            msg_type=Float32,
            topic_name="/temperature",
            publish_rate_hz=2.0,
        )

        # Additional publisher for RViz visualization
        self.marker_pub = self.create_publisher(
            Marker, "/temperature_marker", 10
        )

    def build_message(self) -> Float32:
        msg = Float32()
        msg.data = 20.0 + random.uniform(-1.0, 1.0)

        # Publish RViz marker alongside temperature
        self.publish_marker(msg.data)

        return msg

    def publish_marker(self, temperature: float) -> None:
        marker = Marker()
        marker.header.frame_id = 'map' #"base_link"   # must exist in TF
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "temperature"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position above robot
        marker.pose.position = Point(x=0.0, y=2.0, z=1.0)

        marker.scale.z = 1.0  # text height (meters)

        # White text
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = f"{temperature:.2f} °C"

        self.marker_pub.publish(marker)


def main() -> None:
    TemperaturePublisher.run()


if __name__ == "__main__":
    main()
