from geometry_msgs.msg import Twist


class RecoveryManager:

    def __init__(self, node):

        self.node = node

        self.cmd_vel_pub = node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.active = False
        self.start_time = None

    # =====================================================
    # START
    # =====================================================

    def start_recovery(self):

        self.node.get_logger().warn("Starting recovery")

        self.active = True
        self.start_time = self.node.get_clock().now()

    # =====================================================
    # UPDATE
    # =====================================================

    def update(self):

        if not self.active:
            return False

        twist = Twist()
        twist.angular.z = 0.4

        self.cmd_vel_pub.publish(twist)

        elapsed = (
            self.node.get_clock().now() - self.start_time
        ).nanoseconds * 1e-9

        if elapsed > 5.0:

            self.cmd_vel_pub.publish(Twist())

            self.node.get_logger().info("Recovery complete")

            self.active = False

            return True

        return False