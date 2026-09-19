from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3

from ..base_station_state import BaseStationState
from ..constants import MessageOptions, NAV_TOPIC, DRUM_ROTATION_TOPIC, DRUM_LIFT_TOPIC, \
    PUBLISHER_UPDATE_RATE, INVERT_BACKWARDS_STEERING


class TeleopPublisher(Node):
    """ Publishes teleop commands straight to ROS topics. """
    def __init__(self, state: BaseStationState):
        super().__init__('nav_teleop_publisher')
        self.state = state

        self.publishers_ = {
            NAV_TOPIC: self.create_publisher(Twist, NAV_TOPIC, 10),
            DRUM_ROTATION_TOPIC: self.create_publisher(Float64, DRUM_ROTATION_TOPIC, 10),
            DRUM_LIFT_TOPIC: self.create_publisher(Float64, DRUM_LIFT_TOPIC, 10),
        }
        self.get_logger().info(f'Publishing to {list(self.publishers_.keys())}')

        # Rover mux are set to stop if they don't receive messages for a while, so the most
        # recent state for each topic is republished on a timer regardless of new input.
        self.timer = self.create_timer(1 / PUBLISHER_UPDATE_RATE, self.republish)

    def publish(self, topic_name: str, message_option: MessageOptions, throttle: float) -> None:
        """
        Updates the target state for a topic and publishes it immediately.
        :param topic_name: the name of the topic to publish to.
        :param message_option: the value to modify; a twist attribute, or ignored for float topics.
        :param throttle: the throttle to set the twist option, or float value.
        """
        topic_state = self.state.topic_target_states[topic_name]
        if topic_state['type'] == 'float':
            topic_state['value'] = throttle
        elif topic_state['type'] == 'twist':
            self._update_twist_state(topic_name, topic_state, message_option, throttle)
        else:
            raise ValueError(f"Unsupported message type: {topic_state['type']}")

        self.publishers_[topic_name].publish(self._to_message(topic_state))

    def _update_twist_state(self, topic_name: str, topic_state: dict, message_option: MessageOptions, throttle: float) -> None:
        # Gets the type (linear or angular) and dimension (x, y, or z) to update from the twist option
        _, twist_type, twist_dimension = message_option.value.split('_')

        # For navigation, invert turn throttle if reversing, to allow for more intuitive backwards steering
        if topic_name == NAV_TOPIC:
            is_reversing = topic_state['linear']['x'] < 0
            if INVERT_BACKWARDS_STEERING and message_option == MessageOptions.TWIST_ANGULAR_Z and is_reversing:
                throttle *= -1

        topic_state[twist_type][twist_dimension] = throttle

    def _to_message(self, topic_state: dict):
        if topic_state['type'] == 'float':
            return Float64(data=float(topic_state['value']))

        linear = topic_state['linear']
        angular = topic_state['angular']
        return Twist(
            linear=Vector3(x=float(linear['x']), y=float(linear['y']), z=float(linear['z'])),
            angular=Vector3(x=float(angular['x']), y=float(angular['y']), z=float(angular['z']))
        )

    def republish(self) -> None:
        """ Republish the most recent state for each topic at a fixed rate. """
        for topic_name, topic_state in self.state.topic_target_states.items():
            self.publishers_[topic_name].publish(self._to_message(topic_state))
