import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class ControlsSubscriber(Node):

    def __init__(self):
        super().__init__('controls_subscriber')
        self.get_logger().info('Controls subscriber test node started')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_test',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: linear: {msg.linear}, angular: {msg.angular}')


def main(args=None):
    rclpy.init(args=args)
    controls_subscriber = ControlsSubscriber()
    rclpy.spin(controls_subscriber)

    controls_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
