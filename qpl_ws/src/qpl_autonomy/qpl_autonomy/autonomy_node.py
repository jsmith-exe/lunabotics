import rclpy
from rclpy.node import Node

from qpl_autonomy.navigation_manager import NavigationManager
from qpl_autonomy.recovery_manager import RecoveryManager
from qpl_autonomy.waypoint_manager import WaypointManager


class AutonomyNode(Node):

    def __init__(self):

        super().__init__('autonomy_node')

        # ---------------------------------------------
        # PARAMS
        # ---------------------------------------------

        self.declare_parameter('config_path', '')

        config_path = self.get_parameter(
            'config_path'
        ).get_parameter_value().string_value

        # ---------------------------------------------
        # MANAGERS
        # ---------------------------------------------

        self.waypoints = WaypointManager(config_path)

        self.navigation = NavigationManager(self)

        self.recovery = RecoveryManager(self)

        # ---------------------------------------------
        # FSM
        # ---------------------------------------------

        self.state = "GO_TO_EXCAVATION"

        self.timer = self.create_timer(1.0, self.loop)

        self.get_logger().info("Autonomy node started")

    # =====================================================
    # MAIN LOOP
    # =====================================================

    def loop(self):

        # ---------------------------------------------
        # RECOVERY ACTIVE
        # ---------------------------------------------

        if self.recovery.active:

            finished = self.recovery.update()

            if finished:
                self.state = "GO_TO_EXCAVATION"

            return

        # ---------------------------------------------
        # FSM
        # ---------------------------------------------

        if self.state == "GO_TO_EXCAVATION":

            wp = self.waypoints.get_waypoint(
                "excavation_zone"
            )

            self.navigation.navigate_to(wp)

            self.state = "WAIT_EXCAVATION"

        elif self.state == "WAIT_EXCAVATION":

            if self.navigation.goal_complete:

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

            if self.navigation.goal_complete:

                self.get_logger().info(
                    "Reached construction zone"
                )

                self.state = "GO_TO_EXCAVATION"


# =====================================================
# MAIN
# =====================================================

def main(args=None):

    rclpy.init(args=args)

    node = AutonomyNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()