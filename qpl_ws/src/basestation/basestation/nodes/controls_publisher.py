from threading import Thread
import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3

from basestation.forwarding.tcp_receiver import TCPReceiver
from basestation.constants import NAV_TOPIC, PUBLISHER_UPDATE_RATE, DRUM_ROTATION_TOPIC, DRUM_LIFT_TOPIC

class ControlsPublisher(Node):
    def __init__(self):
        super().__init__('nav_teleop_publisher')

        self.publishers_ = {
            NAV_TOPIC: self.create_publisher(Twist, NAV_TOPIC, 10),
            DRUM_ROTATION_TOPIC: self.create_publisher(Float64, DRUM_ROTATION_TOPIC, 10),
            DRUM_LIFT_TOPIC: self.create_publisher(Float64, DRUM_LIFT_TOPIC, 10)
        }
        topics = list(self.publishers_.keys())
        # Keep track of states for republishing
        self.topic_states = {topic: None for topic in topics}
        self.get_logger().info(f'Publishing to {topics}')

        self.tcp_receiver = TCPReceiver(self.handle_data, log=self.get_logger().info)
        self.tcp_receiver_thread = Thread(target=self.tcp_receiver.start_listening_forever, daemon=True)
        self.tcp_receiver_thread.start()
        self.get_logger().info('TCP Receiver thread started')

        self.timer = self.create_timer(1 / PUBLISHER_UPDATE_RATE, self.republish)

    def handle_data(self, received_data: str, receiver) -> None:
        """
        Handle data sent from the transmitter
        :param received_data: the data sent by the transmitter.
        :param receiver: the receiver instance.
        """
        try:
            self.get_logger().info(f'Received: {received_data}')
            split_data = received_data.split(';') # May be multiple JSON messages, split by ';'
            target_state = json.loads(split_data[len(split_data) - 2]) # second last item is the most recent json data
            topic = target_state['topic']
            type_ = target_state['type']

            if type_ == 'float':
                message_data = Float64(data=float(target_state['value']))
            elif type_ == 'twist':
                message_data = self.twist_from_target_state_dict(target_state)
            else:
                raise ValueError(f"Unsupported message type: {type_}")

            self.topic_states[topic] = message_data
            self.publishers_[topic].publish(message_data)
        except Exception as e:
            self.get_logger().error(f'Error handling data: {e}')

    def twist_from_target_state_dict(self, dict_: dict):
        """
        Convert a dictionary representation of a Twist message into an actual Twist message.
        :param dict_: a dict object with 'linear' and 'angular' keys, each containing a dict with 'x', 'y', 'z' keys,
        each with a numeric value.
        :return:
        """
        linear = dict_['linear']
        angular = dict_['angular']
        return Twist(
            linear=Vector3(x=float(linear['x']), y=float(linear['y']), z=float(linear['z'])),
            angular=Vector3(x=float(angular['x']), y=float(angular['y']), z=float(angular['z']))
        )

    def republish(self):
        """ Republish the most recent messages for each topic at a fixed rate.
        Rover mux are set to stop if they don't receive messages for a while. """
        for topic, msg in self.topic_states.items():
            if msg is None: continue
            self.publishers_[topic].publish(msg)

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
    try:
        main()
    except Exception as e:
        print(e)
        input('Will not recover, press enter to exit.')
