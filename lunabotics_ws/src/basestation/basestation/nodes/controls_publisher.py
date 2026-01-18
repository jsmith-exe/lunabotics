import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from basestation.forwarding.tcp_receiver import TCPReceiver
from threading import Thread


class ControlsPublisher(Node):
    def __init__(self):
        super().__init__('controls_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        self.tcp_receiver = TCPReceiver(self.handle_data)
        self.tcp_receiver_thread = Thread(target=self.tcp_receiver.start_listening_forever, daemon=True)
        self.tcp_receiver_thread.start()
        self.get_logger().info('TCP Receiver thread started')

    def handle_data(self, received_data: str, receiver) -> None:
        """
        Handle data sent from the transmitter
        :param received_data: the data sent by the transmitter.
        :param receiver: the receiver instance.
        """
        msg = String()
        msg.data = received_data
        self.get_logger().info(f'Publishing: {msg.data}')
        self.publisher_.publish(msg)

    def destroy_node(self):
        super().destroy_node()
        self.tcp_receiver.close()

def main(args=None):
    rclpy.init(args=args)
    controls_publisher = ControlsPublisher()
    rclpy.spin(controls_publisher)

    controls_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
