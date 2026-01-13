from __future__ import annotations
from typing import Type, TypeVar
import rclpy
from rclpy.node import Node

T = TypeVar("T", bound="BaseNode")


class BaseNode(Node):
    """Common foundation class for all custom ROS2 nodes."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

    @classmethod
    def run(cls: Type[T], *args, **kwargs) -> None:
        rclpy.init()
        node = cls(*args, **kwargs)

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
