# oo_pub.py
"""
Ultra-simple OO ROS 2 publisher helper (Python, rclpy).
- No background threads; you control spinning via ROS.run().
- Call publishers like functions: pub("hello") or pub({"x":1,"y":0,"z":0})
- Tiny timer helper for periodic publishing.

"""
from __future__ import annotations
from typing import Any, Callable, Optional, Type

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

# ----------------------------
# Optional QoS presets
# ----------------------------
QOS_DEFAULT = QoSProfile(depth=10)
QOS_SENSOR  = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
QOS_LATCHED = QoSProfile(  # new subscribers receive the last published sample
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ----------------------------
# Helpers
# ----------------------------
def _to_msg(msg_type: Type, data: Any):
    """
    Convert common Python data into a ROS message instance.
      - existing message -> returned as-is
      - dict -> msg_type(**data)
      - scalar -> for single-field 'data' messages (String, Float32, etc.)
    """
    if isinstance(data, msg_type):
        return data
    if isinstance(data, dict):
        return msg_type(**data)
    # scalar convenience for single 'data' field messages
    try:
        fields = list(msg_type.get_fields_and_field_types().keys())
        if len(fields) == 1 and fields[0] == "data":
            m = msg_type()
            setattr(m, "data", data)
            return m
    except Exception:
        pass
    # last resort (will raise if not kwargs-compatible)
    return msg_type(**data)

class _TypedPublisher:
    def __init__(self, node: Node, topic: str, msg_type: Type, qos: Optional[QoSProfile] = None):
        self._node = node
        self._msg_type = msg_type
        self._pub = node.create_publisher(msg_type, topic, qos or QOS_DEFAULT)
    def __call__(self, data: Any):
        self._pub.publish(_to_msg(self._msg_type, data))

class Publishers:
    """
    Convenience factory for common messages.
    Extend by adding another 3-liner like Pose/Twist/Imu if you want.
    """
    def __init__(self, node: Node):
        self._node = node

    def typed(self, topic: str, msg_type: Type, qos: Optional[QoSProfile] = None):
        return _TypedPublisher(self._node, topic, msg_type, qos)

    def String(self, topic: str, qos: Optional[QoSProfile] = None):
        from std_msgs.msg import String
        return _TypedPublisher(self._node, topic, String, qos)

    def Float32(self, topic: str, qos: Optional[QoSProfile] = None):
        from std_msgs.msg import Float32
        return _TypedPublisher(self._node, topic, Float32, qos)

    def Vector3(self, topic: str, qos: Optional[QoSProfile] = None):
        from geometry_msgs.msg import Vector3
        return _TypedPublisher(self._node, topic, Vector3, qos)

class ROS:
    """
    Minimal ROS 2 app runner.
    - Use .publishers to create callable publishers.
    - Use .timer(period, fn) to schedule periodic publishing.
    - Call .run(seconds=None) to spin (blocking). If seconds is None, spins forever.
    - Context-managed for clean shutdown.
    """
    def __init__(self, node_name: str = "oo_pub"):
        rclpy.init()
        self._node = Node(node_name)
        self.publishers = Publishers(self._node)
        self._timers = []

    # --- timers ---
    def timer(self, period_sec: float, fn: Callable[[], None]):
        t = self._node.create_timer(period_sec, fn)
        self._timers.append(t)
        return t

    # --- run loop ---
    def run(self, seconds: float | None = None):
        if seconds is None:
            rclpy.spin(self._node)
            return

        end_time = self._node.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self._node.get_clock().now() < end_time:
            rclpy.spin_once(self._node, timeout_sec=0.1)

    # --- logging ---
    @property
    def log(self):
        return self._node.get_logger()

    # --- cleanup ---
    def close(self):
        for t in self._timers:
            try:
                self._node.destroy_timer(t)
            except Exception:
                pass
        self._timers.clear()
        try:
            self._node.destroy_node()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    # context manager support
    def __enter__(self): return self
    def __exit__(self, et, ev, tb): self.close()
