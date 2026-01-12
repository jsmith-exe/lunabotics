# In basestation/basestation/test.py
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    # Your test code here
    node = Node('test_node')
    node.get_logger().info('Test node started!')

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()