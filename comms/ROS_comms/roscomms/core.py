from __future__ import annotations
from typing import Any, Callable, Optional, Type, Dict
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from .adapters import to_msg

class _TypedPublisher:
    def __init__(self, node: Node, topic: str, msg_type: Type, qos: Optional[QoSProfile]):
        self._node = node
        self._msg_type = msg_type
        self._pub = node.create_publisher(msg_type, topic, qos or 10)
    def __call__(self, data: Any):
        self._pub.publish(to_msg(self._msg_type, data))

class ROS:
    """
    Minimal ROS context manager. No background threads.
    You call .run(seconds) to spin (blocking) for timers/flush.
    """
    def __init__(self, node_name: str = "roscomms"):
        rclpy.init()
        self._node = Node(node_name)
        self._timers = []

    @property
    def node(self) -> Node:
        return self._node

    def timer(self, period_sec: float, fn: Callable[[], None]):
        t = self._node.create_timer(period_sec, fn)
        self._timers.append(t)
        return t

    def run(self, seconds: float | None = None):
        if seconds is None:
            rclpy.spin(self._node)
            return
        end = self._node.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self._node.get_clock().now() < end:
            rclpy.spin_once(self._node, timeout_sec=0.1)

    @property
    def log(self):
        return self._node.get_logger()

    def close(self):
        for t in self._timers:
            try: self._node.destroy_timer(t)
            except Exception: pass
        self._timers.clear()
        try: self._node.destroy_node()
        finally:
            try: rclpy.shutdown()
            except Exception: pass

    def __enter__(self): return self
    def __exit__(self, et, ev, tb): self.close()
