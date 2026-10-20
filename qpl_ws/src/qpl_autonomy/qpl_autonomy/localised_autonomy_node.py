import rclpy
from rclpy.time import Time

import tf2_ros

from qpl_autonomy.autonomy_node import AutonomyNode


class LocalisedAutonomyNode(AutonomyNode):
    """Blind autonomy where drive distances are measured with the localisation stack.

    The pose of the base frame in the global frame (map -> base_footprint by
    default) is fed to the sequences in place of drivetrain odometry.
    """

    def __init__(self):
        super().__init__()

        # Parameters
        self.declare_parameter("localisation.global_frame", "map")
        self.declare_parameter("localisation.base_frame", "base_footprint")
        self.declare_parameter("localisation.max_pose_age_s", 1.0)
        self.global_frame = self.get_parameter("localisation.global_frame").value
        self.base_frame = self.get_parameter("localisation.base_frame").value
        self.max_pose_age_s = self.get_parameter("localisation.max_pose_age_s").value

        # Feedback
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"Drive distances measured from {self.global_frame} -> {self.base_frame}."
        )

    """Feedback"""
    def odom_callback(self, msg):
        # Drivetrain odometry is not used
        pass

    def refresh_pose(self):
        # Store the latest localised position, or None if it is unavailable or stale
        self.odom_x = None
        self.odom_y = None

        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warning(f"No localised pose: {e}", throttle_duration_sec=2.0)
            return

        age_s = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds * 1e-9
        if age_s > self.max_pose_age_s:
            self.get_logger().warning(f"Localised pose is stale ({age_s:.1f} s old)", throttle_duration_sec=2.0)
            return

        self.odom_x = transform.transform.translation.x
        self.odom_y = transform.transform.translation.y

    def has_pose(self):
        return self.odom_x is not None and self.odom_y is not None

    """Basestation commands"""
    def handle_excavate(self):
        self.refresh_pose()
        if self.state == "IDLE" and not self.has_pose():
            self.get_logger().warning("Cannot start excavation without a localised pose.")
            return
        super().handle_excavate()

    def handle_deposit(self):
        self.refresh_pose()
        if self.state == "IDLE" and not self.has_pose():
            self.get_logger().warning("Cannot start deposition without a localised pose.")
            return
        super().handle_deposit()

    """FSM update"""
    def update_sequence(self):
        self.refresh_pose()
        if self.state != "IDLE" and not self.has_pose():
            # Without a pose a drive stage would never finish, so stop rather than drive blind
            self.get_logger().error("Lost localised pose, stopping autonomy.")
            self.handle_stop()
            return
        super().update_sequence()


def main(args=None):
    rclpy.init(args=args)

    node = LocalisedAutonomyNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
