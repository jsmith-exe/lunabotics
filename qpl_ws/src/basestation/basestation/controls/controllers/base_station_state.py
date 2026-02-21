from dataclasses import dataclass, field

from ..control_maps import default_control_map

def generate_base_twist_state():
    return {
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    }

@dataclass
class BaseStationState:
    """ Stores the state of the base station, for communication between instances. """
    desktop_controller_enabled: bool = False # Not yet used
    physical_controller_enabled: bool = False # Not yet used
    control_map: dict = field(default_factory=lambda: default_control_map)
    nav_state: dict = field(default_factory=generate_base_twist_state)
