import json
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState

from ..base_station_state import BaseStationState
from ..constants import MessageOptions, NAV_TOPIC, DRUM_ROTATION_TOPIC, DRUM_LIFT_TOPIC, \
    AUTONOMY_COMMAND_TOPIC, NAV_AUTONOMY_COMMAND_TOPIC, NAV_AUTONOMY_STATE_TOPIC, PUBLISHER_UPDATE_RATE, \
    INVERT_BACKWARDS_STEERING, TANK_WHEEL_SEPARATION, JOINT_STATES_TOPIC, DRUM_LIFT_JOINTS

TANK_SIDES = {MessageOptions.TANK_LEFT: 'left', MessageOptions.TANK_RIGHT: 'right'}


def tank_to_twist(left: float, right: float, wheel_separation: float = TANK_WHEEL_SEPARATION) -> tuple[float, float]:
    """
    Converts left and right side speeds to (linear x, angular z). This is the inverse of diff_cont's own conversion,
    so the rover's sides receive exactly these speeds, as long as wheel_separation matches diff_cont's.
    """
    return (right + left) / 2, (right - left) / wheel_separation


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
        self.autonomy_command_publisher = self.create_publisher(String, AUTONOMY_COMMAND_TOPIC, 10)
        self.nav_autonomy_command_publisher = self.create_publisher(String, NAV_AUTONOMY_COMMAND_TOPIC, 10)
        self.get_logger().info(f'Publishing to {list(self.publishers_.keys()) + [AUTONOMY_COMMAND_TOPIC, NAV_AUTONOMY_COMMAND_TOPIC]}')

        # Rover mux are set to stop if they don't receive messages for a while, so the most
        # recent state for each topic is republished on a timer regardless of new input.
        self.timer = self.create_timer(1 / PUBLISHER_UPDATE_RATE, self.republish)

        # Nav autonomy's state, for the teleop window's status line (latched by the rover)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, NAV_AUTONOMY_STATE_TOPIC, self._nav_autonomy_state, latched)

        # Drum lift position, so enabling teleop starts from the bucket's current height
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._joint_states, qos_profile_sensor_data)

    def publish(self, topic_name: str, message_option: MessageOptions, throttle: float) -> None:
        """
        Updates the target state for a topic and publishes it immediately.
        :param topic_name: the name of the topic to publish to.
        :param message_option: the value to modify; a twist attribute, or ignored for float topics.
        :param throttle: the throttle to set the twist option, or float value.
        """
        if not self.state.teleop_enabled: return

        topic_state = self.state.topic_target_states[topic_name]
        if topic_state['type'] == 'float':
            topic_state['value'] = throttle
        elif topic_state['type'] == 'twist':
            self._update_twist_state(topic_name, topic_state, message_option, throttle)
        else:
            raise ValueError(f"Unsupported message type: {topic_state['type']}")

        self.publishers_[topic_name].publish(self._to_message(topic_state))

    def publish_autonomy_command(self, command: str) -> None:
        """
        Sends a command (e.g. EXCAVATE, DEPOSIT, STOP) to the autonomy node.
        Not gated on teleop being enabled, as autonomy is expected to run while teleop is disabled.
        """
        self.autonomy_command_publisher.publish(String(data=command))
        self.get_logger().info(f'Sent autonomy command: {command}')

    def publish_nav_autonomy_command(self, command: str) -> None:
        """
        Sends a command (NAV_EXCAVATE, NAV_DEPOSIT, AUTO, AUTO_TRENCH, AUTO_BOULDERS, AUTO_TRENCH_BOULDERS, STOP) to the Nav2 autonomy node, which qpl_nav starts.
        Not gated on teleop being enabled, for the same reason as publish_autonomy_command.
        """
        self.nav_autonomy_command_publisher.publish(String(data=command))
        self.get_logger().info(f'Sent nav autonomy command: {command}')

    def _update_twist_state(self, topic_name: str, topic_state: dict, message_option: MessageOptions, throttle: float) -> None:
        # Tank inputs set one side, then the whole drive twist is recomputed from both. No backwards steering
        # inversion, since each side is commanded directly.
        if message_option in TANK_SIDES:
            tank = topic_state['tank']
            tank[TANK_SIDES[message_option]] = throttle
            topic_state['linear']['x'], topic_state['angular']['z'] = tank_to_twist(tank['left'], tank['right'])
            return

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

    def _nav_autonomy_state(self, msg: String) -> None:
        try:
            self.state.nav_autonomy = json.loads(msg.data)
        except ValueError:
            return
        self.state.nav_autonomy_time = time.monotonic()

    def _joint_states(self, msg: JointState) -> None:
        lift = [position for name, position in zip(msg.name, msg.position) if name in DRUM_LIFT_JOINTS]
        if len(lift) == len(DRUM_LIFT_JOINTS):
            self.state.lift_position_m = sum(lift) / len(lift)
            self.state.lift_position_time = time.monotonic()

    def republish(self) -> None:
        """ Republish the most recent state for each topic at a fixed rate. """
        if not self.state.teleop_enabled: return
        for topic_name, topic_state in self.state.topic_target_states.items():
            self.publishers_[topic_name].publish(self._to_message(topic_state))
