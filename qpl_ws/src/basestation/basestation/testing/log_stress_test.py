import rclpy
from rclpy.node import Node

class SpamNode(Node):
    def __init__(self):
        super().__init__('spam_node')
        self.count = 0
        self.create_timer(0.01, self.timer_callback)  # every 100ms = 10 msgs/sec

    def timer_callback(self):
        self.get_logger().info(f'Message number: {self.count}')
        self.count += 1

def main():
    rclpy.init()
    rclpy.spin(SpamNode())
    rclpy.shutdown()
