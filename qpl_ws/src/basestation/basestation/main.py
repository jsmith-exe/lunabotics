from threading import Thread

import rclpy
from rclpy.executors import ExternalShutdownException
import yaml

from .base_station_state import BaseStationState
from .constants import CANBUS_CONFIG_PATH
from .controllers.physical_controller import PhysicalController
from .nodes.teleop_publisher import TeleopPublisher
from .ui.teleop_window import open_teleop_window


def main(args=None):
    with open(CANBUS_CONFIG_PATH, "r") as file:
        canbus_config = yaml.safe_load(file)

    rclpy.init(args=args)
    state = BaseStationState()
    node = TeleopPublisher(state)

    physical_controller = PhysicalController(node.publish, state)

    teleop_node_thread = Thread(target=spin_node, args=(node,), daemon=True)
    teleop_node_thread.start()

    try:
        open_teleop_window(state, node.publish, canbus_config) # Should run in main thread
    except KeyboardInterrupt:
        pass

    # Cleanup
    physical_controller.stop()
    node.destroy_node()
    node.get_logger().info('Teleop node destroyed')
    rclpy.try_shutdown()
    teleop_node_thread.join()


def spin_node(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
