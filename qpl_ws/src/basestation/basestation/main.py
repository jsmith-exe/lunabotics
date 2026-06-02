import json
from time import sleep
from threading import Thread

from .controls.controllers.desktop_controller import DesktopController
from .controls.controllers.physical_controller import PhysicalController
from .controls.controllers.base_station_state import BaseStationState
from .forwarding.tcp_transmitter import TCPTransmitter
from .constants import TwistOptions, INVERT_BACKWARDS_STEERING, NAV_TOPIC
from .ui.indicator import show_warning

state = BaseStationState()

def publish_function(topic_name: str, twist_option: TwistOptions, throttle: float):
    """
    A function that handles sending a JSON-serialised Twist message to the receiver for publishing.
    :param topic_name: the name of the topic to publish to (not used in this case).
    :param twist_option: the twist option to modify.
    :param throttle: the throttle to set the twist option.
    """
    topic_state = state.topic_target_states[topic_name]
    # Gets the type (linear or angular) and dimension (x, y, or z) to update from the twist option
    twist_type, twist_dimension = twist_option.value.split('_')

    # For navigation, invert turn throttle if reversing, to allow for more intuitive backwards steering
    if topic_name == NAV_TOPIC:
        is_reversing = topic_state['linear']['x'] < 0
        if INVERT_BACKWARDS_STEERING and twist_option == TwistOptions.ANGULAR_Z and is_reversing:
            throttle *= -1

    # Update state and send message to receiver
    topic_state[twist_type][twist_dimension] = throttle
    transmitter.send_message(json.dumps(topic_state) + ';', False)

# Establish connection to the receiver
connected = False
while not connected:
    try:
        transmitter = TCPTransmitter()
        connected = True
    except ConnectionRefusedError:
        print("Connection refused, retrying in 3s...")
        sleep(3)

warning_thread = Thread(target=show_warning)
desktop_controller = DesktopController(publish_function, state)
physical_controller = PhysicalController(publish_function, state)
try:
    warning_thread.start()
    input("Press enter to exit...")
except KeyboardInterrupt:
    pass
transmitter.close()
physical_controller.stop()
