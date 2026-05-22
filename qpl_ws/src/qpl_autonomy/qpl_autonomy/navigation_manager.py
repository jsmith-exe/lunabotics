from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


class NavigationManager:

    def __init__(self, node):

        self.node = node

        self.nav_client = ActionClient(
            node,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.goal_complete = False
        self.goal_handle = None

        self.nav_client.wait_for_server()

    # =====================================================
    # SEND GOAL
    # =====================================================

    def navigate_to(self, waypoint):

        self.goal_complete = False

        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()

        pose.pose.position.x = float(waypoint["x"])
        pose.pose.position.y = float(waypoint["y"])

        q = quaternion_from_euler(0, 0, float(waypoint["yaw"]))

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal_msg.pose = pose

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    # =====================================================
    # RESPONSE
    # =====================================================

    def goal_response_callback(self, future):

        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error("Goal rejected")
            return

        self.node.get_logger().info("Goal accepted")

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # =====================================================
    # RESULT
    # =====================================================

    def result_callback(self, future):

        self.node.get_logger().info("Goal completed")

        self.goal_complete = True

    # =====================================================
    # CANCEL
    # =====================================================

    def cancel_goal(self):

        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()