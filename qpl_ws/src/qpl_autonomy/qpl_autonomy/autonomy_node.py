import rclpy
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

import yaml
import os


class AutonomyNode(Node):

    def __init__(self):
        super().__init__('autonomy_node')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('config_path', '')

        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        if not config_path:
            raise RuntimeError("No config_path provided to autonomy node")

        # ---------------- LOAD WAYPOINTS ----------------
        with open(config_path, 'r') as f:
            self.waypoints = yaml.safe_load(f)

        # ---------------- NAV2 CLIENT ----------------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ---------------- FSM STATE ----------------
        self.state = "GO_TO_EXCAVATION"
        self.goal_complete = False

        self.get_logger().info("Autonomy node started")

        # wait for Nav2
        self.nav_client.wait_for_server()

        # main loop
        self.timer = self.create_timer(1.0, self.loop)

    # =========================================================
    # FSM LOOP
    # =========================================================
    def loop(self):

        if self.state == "GO_TO_EXCAVATION":
            self.send_goal("excavation_zone")
            self.state = "WAIT_EXCAVATION"

        elif self.state == "WAIT_EXCAVATION":
            if self.goal_complete:
                self.get_logger().info("Reached excavation zone")
                self.state = "GO_TO_DEPOSITION"
                self.goal_complete = False

        elif self.state == "GO_TO_DEPOSITION":
            self.send_goal("deposition_zone")
            self.state = "WAIT_DEPOSITION"

        elif self.state == "WAIT_DEPOSITION":
            if self.goal_complete:
                self.get_logger().info("Reached deposition zone")
                self.state = "GO_TO_EXCAVATION"
                self.goal_complete = False

    # =========================================================
    # NAV2 GOAL SENDING
    # =========================================================
    def send_goal(self, zone_name):

        wp = self.waypoints[zone_name]

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(wp["x"])
        pose.pose.position.y = float(wp["y"])

        q = quaternion_from_euler(0, 0, float(wp["yaw"]))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal_msg.pose = pose

        self.get_logger().info(f"Sending goal: {zone_name}")

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    # =========================================================
    # GOAL RESPONSE
    # =========================================================
    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2")
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # =========================================================
    # RESULT CALLBACK
    # =========================================================
    def result_callback(self, future):

        result = future.result().result

        self.get_logger().info("Goal completed successfully")

        self.goal_complete = True


# =========================================================
# MAIN
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()