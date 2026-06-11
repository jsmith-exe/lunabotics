from dataclasses import dataclass, field

from ..control_maps import default_control_map
from ...constants import NAV_TOPIC, DRUM_LIFT_TOPIC, DRUM_ROTATION_TOPIC


def generate_base_twist_state(topic):
    return {
        'topic': str(topic),
        'type': 'twist',
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    }

def generate_base_float_state(topic):
    return {
        'topic': str(topic),
        'type': 'float',
        'value': 0.0
    }

def generate_topic_state():
    return {
        NAV_TOPIC: generate_base_twist_state(NAV_TOPIC),
        DRUM_LIFT_TOPIC: generate_base_float_state(DRUM_LIFT_TOPIC),
        DRUM_ROTATION_TOPIC: generate_base_float_state(DRUM_ROTATION_TOPIC),
    }

@dataclass
class BaseStationState:
    """ Stores the state of the base station, for communication between instances. """
    teleop_enabled: bool = True
    control_map: dict = field(default_factory=lambda: default_control_map)
    topic_target_states: dict = field(default_factory=generate_topic_state)
