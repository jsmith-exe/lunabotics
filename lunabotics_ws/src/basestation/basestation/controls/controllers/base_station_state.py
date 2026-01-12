from dataclasses import dataclass, field

from lunabotics_ws.src.basestation.basestation.controls.control_maps import default_desktop_control_map, default_controller_control_map


@dataclass
class BaseStationState:
    """ Stores the state of the base station, for communication between instances. """
    desktop_controller_enabled: bool = False
    physical_controller_enabled: bool = False
    desktop_control_map: dict = field(default_factory=lambda: default_desktop_control_map)
    controller_control_map: dict = field(default_factory=lambda: default_controller_control_map)
