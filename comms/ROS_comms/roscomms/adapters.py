"""
Message adapters: convert Python data -> ROS 2 message instances.
Includes a small registry so you can say "std_msgs/String" in config.
"""
from __future__ import annotations
from typing import Any, Dict, Type
import importlib

# Cache "pkg/MsgName" -> class
_MSG_CACHE: Dict[str, Type] = {}

def resolve_msg(msg_ref: str) -> Type:
    """
    msg_ref examples:
      - "std_msgs/String"
      - "geometry_msgs/Vector3"
      - "sensor_msgs/Imu"
    """
    if msg_ref in _MSG_CACHE:
        return _MSG_CACHE[msg_ref]
    pkg, name = msg_ref.split("/", 1)
    mod = importlib.import_module(f"{pkg}.msg")
    cls = getattr(mod, name)
    _MSG_CACHE[msg_ref] = cls
    return cls

def to_msg(msg_type: Type, data: Any):
    """Accepts existing msg, dict, or scalar for single-field 'data' messages."""
    if isinstance(data, msg_type):
        return data
    if isinstance(data, dict):
        return msg_type(**data)
    # scalar convenience for messages like String/Float32
    try:
        fields = list(msg_type.get_fields_and_field_types().keys())
        if len(fields) == 1 and fields[0] == "data":
            m = msg_type()
            setattr(m, "data", data)
            return m
    except Exception:
        pass
    return msg_type(**data)  # let it raise if incompatible
