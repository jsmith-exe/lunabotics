# temp_publisher.py

import random
from std_msgs.msg import Float32
from base_publisher import BasePublisher


class TemperaturePublisher(BasePublisher):
    def __init__(self) -> None:
        super().__init__(
            node_name="temp_pub",
            msg_type=Float32,
            topic_name="temperature",
            publish_rate_hz=2.0,
        )

    def build_message(self) -> Float32:
        msg = Float32()
        msg.data = 20.0 + random.uniform(-1.0, 1.0)
        return msg


if __name__ == "__main__":
    TemperaturePublisher.run()
