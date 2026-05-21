import rclpy
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from tf_transformations import quaternion_from_euler

import yaml


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

        # ---------------- CMD_VEL (RECOVERY) ----------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------------- FSM STATE ----------------
        self.state = "GO_TO_EXCAVATION"
        self.goal_complete = False

        # ---------------- RECOVERY STATE ----------------
        self.mode = "NORMAL"
        self.goal_handle = None
        self.goal_start_time = None
        self.goal_timeout_sec = 45.0
        self.recovery_start_time = None

        self.get_logger().info("Autonomy node started")

        # wait for Nav2
        self.nav_client.wait_for_server()

        # main loop
        self.timer = self.create_timer(1.0, self.loop)

    # =========================================================
    # MAIN LOOP
    # =========================================================
    def loop(self):

        # ---------------- RECOVERY MODE ----------------
        if self.mode == "WORLD_MODEL_RESET":
            self.run_world_model_reset()
            return

        # ---------------- STUCK DETECTION ----------------
        if self.check_stuck():
            self.enter_world_model_reset()
            return

        # ---------------- NORMAL FSM ----------------
        self.run_fsm()

    # =========================================================
    # FSM
    # =========================================================
    def run_fsm(self):

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
    # GOAL SENDING
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

        self.goal_start_time = self.get_clock().now()

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    # =========================================================
    # GOAL RESPONSE
    # =========================================================
    def goal_response_callback(self, future):

        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2")
            return

        self.get_logger().info("Goal accepted")

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # =========================================================
    # RESULT CALLBACK
    # =========================================================
    def result_callback(self, future):

        self.get_logger().info("Goal completed successfully")
        self.goal_complete = True

    # =========================================================
    # STUCK DETECTION (KEY FIX)
    # =========================================================
    def check_stuck(self):

        if self.mode != "NORMAL":
            return False

        if self.goal_start_time is None:
            return False

        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds * 1e-9

        if elapsed > self.goal_timeout_sec and not self.goal_complete:
            self.get_logger().warn("Robot appears stuck (timeout)")
            return True

        return False

    # =========================================================
    # ENTER RECOVERY
    # =========================================================
    def enter_world_model_reset(self):

        self.get_logger().warn("ENTERING WORLD MODEL RESET")

        self.mode = "WORLD_MODEL_RESET"
        self.recovery_start_time = self.get_clock().now()

        # cancel Nav2 goal
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

        # stop robot immediately
        self.cmd_vel_pub.publish(Twist())

    # =========================================================
    # RECOVERY BEHAVIOUR
    # =========================================================
    def run_world_model_reset(self):

        twist = Twist()
        twist.angular.z = 0.4
        self.cmd_vel_pub.publish(twist)

        elapsed = (self.get_clock().now() - self.recovery_start_time).nanoseconds * 1e-9

        # 5 second scan
        if elapsed > 5.0:

            self.get_logger().info("Recovery complete, resuming navigation")

            # stop motion
            self.cmd_vel_pub.publish(Twist())

            # reset state
            self.mode = "NORMAL"
            self.goal_complete = False
            self.state = "GO_TO_EXCAVATION"
            self.goal_start_time = None


# =========================================================
# MAIN
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()