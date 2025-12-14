from __future__ import annotations
from typing import Type
from rclpy.qos import QoSProfile

from ...base_node import BaseNode


class BaseSubscriber(BaseNode):
    """Reusable OOP base class for a ROS2 subscriber."""

    def __init__(
        self,
        node_name: str,
        msg_type: Type,
        topic_name: str,
        qos_depth: int = 10,
    ) -> None:

        super().__init__(node_name)

        self.msg_type = msg_type
        qos = QoSProfile(depth=qos_depth)

        self.subscription_ = self.create_subscription(
            msg_type,
            topic_name,
            self._callback,
            qos,
        )

    def _callback(self, msg):
        self.handle_message(msg)

    def handle_message(self, msg):
        """Override in subclass"""
        raise NotImplementedError
