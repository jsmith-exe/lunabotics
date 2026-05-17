import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SpamNode(Node):
    def __init__(self):
        super().__init__('spam_node')
        self.count = 0
        self.create_timer(0.01, self.timer_callback)  # every 100ms = 10 msgs/sec
        self.publisher_ = self.create_publisher(String, '/spam_topic', 10)

    def timer_callback(self):
        self.count += 1
        txt = f'Message number: {self.count}'
        self.get_logger().info(txt)

        msg = String()
        msg.data = txt
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(SpamNode())
    rclpy.shutdown()
