import json

from controls.controllers.desktop_controller import DesktopController
from controls.controllers.physical_controller import PhysicalController
from controls.controllers.base_station_state import BaseStationState
from forwarding.tcp_transmitter import TCPTransmitter
from controls.constants import TwistOptions

# TODO move to State object
twist_state = {
    'linear': { 'x': 0.0, 'y': 0.0, 'z': 0.0 },
    'angular': { 'x': 0.0, 'y': 0.0, 'z': 0.0 }
}

def publish_function(topic_name: str, twist_option: TwistOptions, throttle: float):
    """
    A mock publish method that can be used to move the turtle, based on the controllers.
    :param topic_name: the name of the topic to publish to.
    :param twist_option: the twist option to modify.
    :param throttle: the throttle to set the twist option.
    """
    twist_type, twist_dimension = twist_option.value.split('_')
    twist_state[twist_type][twist_dimension] = throttle
    transmitter.send_message(json.dumps(twist_state) + ';', False)

def get_index(args, index, default):
    try:
        return args[index]
    except IndexError:
        return default


state = BaseStationState()
transmitter = TCPTransmitter()
desktop_controller = DesktopController(publish_function, state)
physical_controller = PhysicalController(publish_function, state)
try:
    input("Press enter to exit...")
except KeyboardInterrupt:
    pass
transmitter.close()
physical_controller.stop()
