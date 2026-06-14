import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from datetime import datetime
import os

SEVERITY = {
    10: "DEBUG",
    20: "INFO",
    30: "WARN",
    40: "ERROR",
    50: "FATAL",
}

OUTPUT_FILE = os.path.expanduser("~/rosout_combined.log")


class RosoutLogger(Node):
    def __init__(self):
        super().__init__("rosout_logger")

        self._log_file = open(OUTPUT_FILE, "a", buffering=1) # buffering=1 writes line by line as received

        session_start = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._write(f"{'=' * 72}")
        self._write(f"  Session started: {session_start}")
        self._write(f"{'=' * 72}")

        self.get_logger().info(f"rosout_logger: writing to {OUTPUT_FILE}")
        self._log_subscription = self.create_subscription(Log, "/rosout", self._callback, 100)

    def _callback(self, msg: Log):
        stamp_sec = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        timestamp = datetime.fromtimestamp(stamp_sec).strftime("%H:%M:%S.%f")[:-3]
        severity = SEVERITY.get(msg.level)

        self._write(f"[{timestamp}] [{severity}] [{msg.name}] {msg.msg}")

    def _write(self, text: str):
        print(text, file=self._log_file)

    def destroy_node(self):
        session_end = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._write(f"{'=' * 72}")
        self._write(f"  Session ended:   {session_end}")
        self._write(f"{'=' * 72}\n")
        self._log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosoutLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(e)
    print("rosout_logger: shutdown complete")


if __name__ == "__main__":
    main()