from __future__ import annotations
from typing import Optional, Type, Dict, Any
from rclpy.qos import QoSProfile
from .adapters import resolve_msg
from .core import _TypedPublisher, ROS

class PublisherFactory:
    """
    Reusable factory that creates callable publishers by:
      - explicit (topic, msg_cls), or
      - config mapping (topic -> "pkg/Msg")
    """
    def __init__(self, ros: ROS):
        self._ros = ros
        self._cache: Dict[tuple[str, Type], _TypedPublisher] = {}

    def typed(self, topic: str, msg_type: Type, qos: Optional[QoSProfile] = None):
        key = (topic, msg_type)
        if key not in self._cache:
            self._cache[key] = _TypedPublisher(self._ros.node, topic, msg_type, qos)
        return self._cache[key]

    def from_string(self, topic: str, msg_ref: str, qos: Optional[QoSProfile] = None):
        return self.typed(topic, resolve_msg(msg_ref), qos)

    def from_config(self, mapping: Dict[str, Any], qos_map: Dict[str, QoSProfile] | None = None):
        """
        mapping example (from YAML):
            topics:
              /status: std_msgs/String
              /temperature: std_msgs/Float32
        Returns dict: topic -> callable publisher
        """
        pubs = {}
        for topic, ref in mapping.items():
            if isinstance(ref, dict):
                msg_ref = ref["type"]
                qos = None
                if qos_map and (q := ref.get("qos")):
                    qos = qos_map.get(q)
                pubs[topic] = self.from_string(topic, msg_ref, qos)
            else:
                pubs[topic] = self.from_string(topic, ref, None)
        return pubs
