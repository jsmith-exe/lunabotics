"""Publishes the competition arena zones as a MarkerArray for RViz.

Draws translucent colored rectangles on the arena floor (map frame) so the
teleoperator can see at a glance which zone the rover is in, plus thin outlines
for the arena boundary and the berm deposition target. Static overlay: no TF
lookup needed, the markers are fixed in the map frame. Text labels and waypoint
arrows are intentionally omitted to keep the view uncluttered.

Topic: /zone_overlay  ->  add an rviz_default_plugins/MarkerArray display.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import os
import yaml
from ament_index_python.packages import get_package_share_directory


ZONE_COLORS = {
    "START": (0.20, 0.80, 0.30, 0.30),
    "CONSTRUCTION": (0.25, 0.45, 0.95, 0.30),
    "EXCAVATION": (0.95, 0.30, 0.70, 0.30),
    "BERM": (0.1, 1.0, 0.2, 1.0),
}

FRAME = "map"


class ZoneOverlay(Node):
    def __init__(self):
        super().__init__("zone_overlay")
        package_path = get_package_share_directory("qpl_rover")

        selector_path = os.path.join(
            package_path,
            "config",
            "arena",
            "selector.yaml"
        )

        with open(selector_path, "r") as file:
            selector_config = yaml.safe_load(file)

        arena_name = selector_config["arena"]

        arena_config_path = os.path.join(
            package_path,
            "config",
            "arena",
            f"{arena_name}.yaml"
        )

        with open(arena_config_path, "r") as file:
            arena_config = yaml.safe_load(file)["arena"]

        self.arena_width = arena_config["width"]
        self.arena_length = arena_config["length"]
        self.zones = arena_config["zones"]

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
        for zone in self.zones:
            name = zone["name"]

            arr.markers.append(
                self._fill(
                    mid,
                    zone["x_min"],
                    zone["x_max"],
                    zone["y_min"],
                    zone["y_max"],
                    ZONE_COLORS[name]
                )
            )
            mid += 1

        # Arena boundary
        arr.markers.append(
            self._outline(
                mid,
                0.0,
                self.arena_width,
                0.0,
                self.arena_length,
                (1.0, 1.0, 1.0, 0.8)
            )
        )
        mid += 1

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
