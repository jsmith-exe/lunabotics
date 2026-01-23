from __future__ import annotations
from typing import Type
from rclpy.qos import QoSProfile

from .base_node import BaseNode


class BasePublisher(BaseNode):
    """Reusable OOP base class for a timed ROS2 publisher."""

    def __init__(
        self,
        node_name: str,
        msg_type: Type,
        topic_name: str,
        publish_rate_hz: float,
        qos_depth: int = 10,
    ) -> None:

        super().__init__(node_name)

        self.msg_type = msg_type
        qos = QoSProfile(depth=qos_depth)

        self.publisher_ = self.create_publisher(msg_type, topic_name, qos)

        period_sec = 1.0 / publish_rate_hz
        self.timer = self.create_timer(period_sec, self._publish)

    def _publish(self) -> None:
        msg = self.build_message()
        self.publisher_.publish(msg)

    def build_message(self):
        raise NotImplementedError
