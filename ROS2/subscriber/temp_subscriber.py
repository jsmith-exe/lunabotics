# python3 -m lunabotics.ROS2.subscriber.temp_subscriber

from std_msgs.msg import Float32
from .base_subscriber import BaseSubscriber
import threading
from typing import Optional, Tuple


class TemperatureListener(BaseSubscriber):
    def __init__(self) -> None:
        super().__init__(
            node_name="temp_listener",
            msg_type=Float32,
            topic_name="temperature",
        )

        self._lock = threading.Lock()
        self._latest_temp: Optional[float] = None
        self._latest_stamp = None

    def handle_message(self, msg: Float32) -> None:
        with self._lock:
            self._latest_temp = float(msg.data)
            self._latest_stamp = self.get_clock().now()

        self.get_logger().info(
            f"Received temperature: {msg.data:.2f} °C"
        )

    def get_latest_temperature(self) -> Tuple[Optional[float], Optional[object]]:
        """
        Returns:
            (temperature, timestamp)
            If no message received yet → (None, None)
        """
        with self._lock:
            return self._latest_temp, self._latest_stamp


if __name__ == "__main__":
    TemperatureListener.run()
