import json

from controls.controllers.desktop_controller import DesktopController
from controls.controllers.base_station_state import BaseStationState
from controls.constants import Commands
from forwarding.tcp_transmitter import TCPTransmitter

# TODO move to State object
twist_state = {
    'linear': { 'x': 0, 'y': 0, 'z': 0 },
    'angular': { 'x': 0, 'y': 0, 'z': 0 }
}

def publish_function(command: str, *args):
    default = 1

    thrust = None
    turning_thrust = None
    match command:
        case Commands.FORWARD:
            thrust = get_index(args, 0, default)
        case Commands.REVERSE:
            thrust = -get_index(args, 0, default)
        case Commands.LEFT:
            turning_thrust = -get_index(args, 0, default)
        case Commands.RIGHT:
            turning_thrust = get_index(args, 0, default)
        case Commands.STOP_SIGNAL:
            command = args[0]
            if command == Commands.FORWARD or command == Commands.REVERSE: thrust = 0
            else: turning_thrust = 0

    if thrust is not None: twist_state['linear']['x'] = thrust
    if turning_thrust is not None: twist_state['angular']['x'] = turning_thrust

    transmitter.send_message(json.dumps(twist_state), False)

def get_index(args, index, default):
    try:
        return args[index]
    except IndexError:
        return default


state = BaseStationState()
transmitter = TCPTransmitter()
desktop_controller = DesktopController(publish_function, state)

input("Press enter to exit...")
transmitter.close()
