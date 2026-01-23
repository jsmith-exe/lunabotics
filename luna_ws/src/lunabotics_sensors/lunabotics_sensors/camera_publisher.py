import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .base_publisher import BasePublisher


class CameraPublisher(BasePublisher):
    def __init__(self) -> None:
        self.bridge = CvBridge()

        super().__init__(
            node_name="camera_pub",
            msg_type=Image,
            topic_name="/camera/image",
            publish_rate_hz=10.0,
        )

    def build_message(self) -> Image:
        # generate a test frame (480x640 RGB)
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        return self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")


def main() -> None:
    CameraPublisher.run()


if __name__ == "__main__":
    main()
