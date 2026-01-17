import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('controls_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

    def handle_data(self, received_data: bytes) -> None:
        """
        Handle data sent from the transmitter
        :param received_data: the data sent by the transmitter.
        """
        received_data = received_data.decode('utf-8')
        msg = String()
        msg.data = received_data
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
