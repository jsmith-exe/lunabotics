import json
from time import sleep

from .controls.controllers.desktop_controller import DesktopController
from .controls.controllers.physical_controller import PhysicalController
from .controls.controllers.base_station_state import BaseStationState
from .forwarding.tcp_transmitter import TCPTransmitter
from .constants import TwistOptions, INVERT_BACKWARDS_STEERING

state = BaseStationState()

def publish_function(topic_name: str, twist_option: TwistOptions, throttle: float):
    """
    A function that handles sending a JSON-serialised Twist message to the receiver for publishing.
    :param topic_name: the name of the topic to publish to (not used in this case).
    :param twist_option: the twist option to modify.
    :param throttle: the throttle to set the twist option.
    """
    twist_type, twist_dimension = twist_option.value.split('_')
    is_reversing = state.nav_state['linear']['x'] < 0
    if INVERT_BACKWARDS_STEERING and twist_option == TwistOptions.ANGULAR_Z and is_reversing:
        throttle *= -1
    state.nav_state[twist_type][twist_dimension] = throttle
    transmitter.send_message(json.dumps(state.nav_state) + ';', False)

# Establish connection to the receiver
connected = False
while not connected:
    try:
        transmitter = TCPTransmitter()
        connected = True
    except ConnectionRefusedError:
        print("Connection refused, retrying in 3s...")
        sleep(3)

desktop_controller = DesktopController(publish_function, state)
physical_controller = PhysicalController(publish_function, state)
try:
    input("Press enter to exit...")
except KeyboardInterrupt:
    pass
transmitter.close()
physical_controller.stop()
