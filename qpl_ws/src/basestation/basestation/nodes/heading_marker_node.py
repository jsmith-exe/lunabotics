"""Publishes a heading arrow on the rover so its front is obvious in RViz.

The rover is close to square from above and the TF axes are small, so in the
top-down map and the chase views it is easy to lose track of which end is the
front. This draws a bright arrow along +x of base_footprint (the end the front
camera is mounted on), floating above the frame so neither the robot model nor
the floor hides it, plus a FRONT label at the tip.

The markers are frame-locked to base_footprint with a zero stamp, so RViz moves
them with the latest TF and they follow the rover's roll, pitch and yaw. Only
the TF is needed - no odometry topic or Nav2.

Topic: /heading_marker  ->  add an rviz_default_plugins/MarkerArray display.
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


FRAME = "base_footprint"

# The base frame is 1.17 m long and its top sits at about 0.55 m, so the arrow
# starts at the centre, clears the front bumper by about 0.5 m and floats over
# the top frame.
ARROW_LENGTH = 1.1
ARROW_Z = 0.75
COLOR = (1.0, 0.85, 0.0, 1.0)


class HeadingMarker(Node):
    def __init__(self):
        super().__init__("heading_marker")
        self.pub = self.create_publisher(MarkerArray, "/heading_marker", 1)
        # Republish periodically so RViz always catches the markers regardless
        # of when its subscription comes up.
        self.create_timer(1.0, self._publish)
        self.get_logger().info("Heading marker publishing on /heading_marker")

    def _base(self, mid, mtype):
        m = Marker()
        m.header.frame_id = FRAME
        # Zero stamp: RViz uses the latest TF instead of waiting for one that
        # matches this time, which also keeps it independent of use_sim_time.
        m.header.stamp = Time()
        m.ns = "heading"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.frame_locked = True
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = COLOR
        return m

    def _publish(self):
        arrow = self._base(0, Marker.ARROW)
        arrow.points = [
            Point(x=0.0, y=0.0, z=ARROW_Z),
            Point(x=ARROW_LENGTH, y=0.0, z=ARROW_Z),
        ]
        # With points set: shaft diameter, head diameter, head length.
        arrow.scale.x = 0.08
        arrow.scale.y = 0.22
        arrow.scale.z = 0.25

        label = self._base(1, Marker.TEXT_VIEW_FACING)
        label.pose.position.x = ARROW_LENGTH + 0.15
        label.pose.position.z = ARROW_Z + 0.15
        label.scale.z = 0.18
        label.text = "FRONT"

        self.pub.publish(MarkerArray(markers=[arrow, label]))


def main(args=None):
    rclpy.init(args=args)
    node = HeadingMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
