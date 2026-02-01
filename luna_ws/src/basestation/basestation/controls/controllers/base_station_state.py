from dataclasses import dataclass, field

from ..control_maps import default_control_map


@dataclass
class BaseStationState:
    """ Stores the state of the base station, for communication between instances. """
    desktop_controller_enabled: bool = False
    physical_controller_enabled: bool = False
    control_map: dict = field(default_factory=lambda: default_control_map)
