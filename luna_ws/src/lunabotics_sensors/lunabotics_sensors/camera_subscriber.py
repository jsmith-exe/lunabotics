# python3 -m lunabotics.ROS2.subscriber.camera_subscriber

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .base_subscriber import BaseSubscriber

class CameraListener(BaseSubscriber):
    def __init__(self) -> None:
        self.bridge = CvBridge()

        super().__init__(
            node_name="camera_listener",
            msg_type=Image,
            topic_name="camera/image",
        )

    def handle_message(self, msg: Image) -> None:
        # just log receipt, no GUI required
        self.get_logger().info("Received test camera frame")


if __name__ == "__main__":
    CameraListener.run()
