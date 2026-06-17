"""Publishes the competition arena zones as a MarkerArray for RViz.

Draws translucent colored rectangles on the arena floor (map frame) so the
teleoperator can see at a glance which zone the rover is in, plus thin outlines
for the arena boundary and the berm deposition target. Static overlay: no TF
lookup needed, the markers are fixed in the map frame. Text labels and waypoint
arrows are intentionally omitted to keep the view uncluttered.

Map-frame arena: X(0..4.4) Y(0..7.9), origin at the AprilTag corner.
Topic: /zone_overlay  ->  add an rviz_default_plugins/MarkerArray display.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# name, x_min, x_max, y_min, y_max, (r, g, b, a)
ZONES = [
    ("START",        0.0, 2.0,  0.0,   2.0,    (0.20, 0.80, 0.30, 0.30)),
    ("CONSTRUCTION", 2.9, 4.4,  0.0,   2.6,    (0.25, 0.45, 0.95, 0.30)),
    ("EXCAVATION",   0.0, 4.4,  5.275, 7.9,    (0.95, 0.30, 0.70, 0.30)),
]

# Berm deposition target (map frame) — handy reference while depositing.
BERM_CENTER = (3.5, 1.0)
BERM_SIZE = (0.9, 1.4)  # x, y footprint of the target berm area

FRAME = "map"


class ZoneOverlay(Node):
    def __init__(self):
        super().__init__("zone_overlay")
        self.pub = self.create_publisher(MarkerArray, "/zone_overlay", 1)
        # Republish periodically so RViz always catches the markers regardless
        # of when its subscription comes up.
        self.create_timer(1.0, self._publish)
        self.get_logger().info("Zone overlay publishing on /zone_overlay")

    def _base(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.frame_locked = True
        m.pose.orientation.w = 1.0
        return m

    def _fill(self, mid, x0, x1, y0, y1, rgba, z=0.02):
        m = self._base("zone_fill", mid, Marker.CUBE)
        m.pose.position.x = (x0 + x1) / 2.0
        m.pose.position.y = (y0 + y1) / 2.0
        m.pose.position.z = z / 2.0
        m.scale.x = x1 - x0
        m.scale.y = y1 - y0
        m.scale.z = z
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _outline(self, mid, x0, x1, y0, y1, rgba, width=0.04, z=0.05):
        m = self._base("zone_outline", mid, Marker.LINE_STRIP)
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        m.points = [Point(x=cx, y=cy, z=z) for cx, cy in corners]
        return m

    def _publish(self):
        arr = MarkerArray()

        # Tell RViz to drop everything it is still holding before we re-add the
        # current markers. Without this, markers we stop publishing (old labels,
        # the removed OBSTACLE zone, or fills that changed id between runs) stay
        # on screen and can overlap/z-fight into a flicker. DELETEALL is the
        # first entry, so it is processed before the ADDs in the same message
        # and does not itself cause a flash.
        clear = Marker()
        clear.header.frame_id = FRAME
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        mid = 0
        for name, x0, x1, y0, y1, rgba in ZONES:
            arr.markers.append(self._fill(mid, x0, x1, y0, y1, rgba))
            mid += 1

        # Arena boundary
        arr.markers.append(
            self._outline(mid, 0.0, 4.4, 0.0, 7.9, (1.0, 1.0, 1.0, 0.8))
        )
        mid += 1

        # Berm deposition target
        bx, by = BERM_CENTER
        bw, bh = BERM_SIZE
        arr.markers.append(
            self._outline(mid, bx - bw / 2, bx + bw / 2, by - bh / 2,
                          by + bh / 2, (0.1, 1.0, 0.2, 1.0), width=0.05, z=0.06)
        )

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ZoneOverlay()
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
