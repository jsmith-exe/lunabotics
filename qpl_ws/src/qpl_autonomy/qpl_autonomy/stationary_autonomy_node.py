import rclpy

from qpl_autonomy.timed_autonomy_node import TimedAutonomyNode, EXCAVATION_KEYS
from qpl_autonomy.timed_sequences import StationaryExcavationSequence


class StationaryAutonomyNode(TimedAutonomyNode):
    """Timed blind autonomy where the rover doesn't move while excavating.

    Deposition is the same as timed_autonomy_node.
    """

    def __init__(self):
        super().__init__()

        excavation_config = {
            key: self.get_parameter(f"excavation.{key}").value for key in EXCAVATION_KEYS
        }

        # Sequences
        self.excavation = StationaryExcavationSequence(
            cmd_vel_pub=self.cmd_vel_pub,
            drum_lift_pub=self.drum_lift_pub,
            drum_spin_pub=self.drum_spin_pub,
            config=excavation_config,
            logger=self.get_logger(),
        )

        self.get_logger().info("Excavation is stationary.")


def main(args=None):
    rclpy.init(args=args)

    node = StationaryAutonomyNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
