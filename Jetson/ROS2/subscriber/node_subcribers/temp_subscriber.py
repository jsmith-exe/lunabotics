from std_msgs.msg import Float32
from ..base.base_subscriber import BaseSubscriber


class TemperatureListener(BaseSubscriber):
    def __init__(self) -> None:
        super().__init__(
            node_name="temp_listener",
            msg_type=Float32,
            topic_name="temperature",
        )

    def handle_message(self, msg: Float32) -> None:
        self.get_logger().info(
            f"Received temperature: {msg.data:.2f} °C"
        )


if __name__ == "__main__":
    TemperatureListener.run()
