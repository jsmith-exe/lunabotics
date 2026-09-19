from threading import Thread

import rclpy
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

    window_thread = Thread(target=open_teleop_window, args=(state, node.publish, canbus_config), daemon=True)
    physical_controller = PhysicalController(node.publish, state)

    window_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    physical_controller.stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
