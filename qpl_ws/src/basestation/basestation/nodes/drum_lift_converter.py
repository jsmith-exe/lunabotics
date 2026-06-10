import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

class DrumLiftBridge(Node):
    def __init__(self):
        super().__init__('drum_lift_converter')
        self.sub = self.create_subscription(
            Float64,
            'cmd_drum_lift',
            self.callback,
            10)
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/drum_lift_cont/commands',
            10)

    def callback(self, msg):
        out = Float64MultiArray()
        normalised_for_weird_convention = msg.data / 2 + 0.5
        out.data = [normalised_for_weird_convention, normalised_for_weird_convention]  # [msg.data, msg.data] when second actuator added
        self.pub.publish(out)
        self.get_logger().info(f'Converted {msg.data} to {out.data}')

def main():
    rclpy.init()
    rclpy.spin(DrumLiftBridge())

if __name__ == '__main__':
    main()