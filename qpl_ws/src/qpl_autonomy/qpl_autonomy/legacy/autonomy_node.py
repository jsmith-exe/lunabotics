import rclpy
from rclpy.node import Node

from qpl_autonomy.navigation_manager import NavigationManager
from qpl_autonomy.waypoint_manager import WaypointManager
from qpl_autonomy.excavation_node import MAX_NAV_RETRIES  # single source of truth

class AutonomyNode(Node):
    def __init__(self):
        super().__init__('autonomy_node')

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter(
            'config_path'
        ).get_parameter_value().string_value

        self.waypoints = WaypointManager(config_path)
        self.navigation = NavigationManager(self)
        self._nav_retry = 0

        self.state = "GO_TO_EXCAVATION"
        self.timer = self.create_timer(1.0, self.loop)
        self.get_logger().info("Autonomy node started")

    def _nav_done(self, waypoint):
        """True only once navigation to `waypoint` SUCCEEDED (FSM may advance).
        On a failed/aborted/rejected/timed-out goal, re-issues it up to
        MAX_NAV_RETRIES times, then holds rather than treating it as arrival.
        Returns False while navigating/retrying/held."""
        if not self.navigation.goal_complete:
            return False
        if self.navigation.goal_succeeded:
            self._nav_retry = 0
            return True
        self._nav_retry += 1
        if self._nav_retry <= MAX_NAV_RETRIES:
            self.get_logger().warn(
                f"Navigation failed — retry {self._nav_retry}/{MAX_NAV_RETRIES}"
            )
            self.navigation.navigate_to(waypoint)
        else:
            self.get_logger().error("Navigation failed after retries — holding")
            self.state = "HOLD"
        return False

    def loop(self):
        if self.state == "GO_TO_EXCAVATION":
            wp = self.waypoints.get_waypoint(
                "excavation_zone"
            )
            self.navigation.navigate_to(wp)
            self.state = "WAIT_EXCAVATION"

        elif self.state == "WAIT_EXCAVATION":
            if self._nav_done(self.waypoints.get_waypoint("excavation_zone")):
                self.get_logger().info(
                    "Reached excavation zone"
                )
                self.state = "GO_TO_CONSTRUCTION"

        elif self.state == "GO_TO_CONSTRUCTION":
            wp = self.waypoints.get_waypoint(
                "construction_zone"
            )
            self.navigation.navigate_to(wp)
            self.state = "WAIT_CONSTRUCTION"

        elif self.state == "WAIT_CONSTRUCTION":
            if self._nav_done(self.waypoints.get_waypoint("construction_zone")):
                self.get_logger().info(
                    "Reached construction zone"
                )
                self.state = "GO_TO_EXCAVATION"

        elif self.state == "HOLD":
            self.get_logger().error(
                "Navigation aborted — holding for operator intervention",
                throttle_duration_sec=5.0,
            )

def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()