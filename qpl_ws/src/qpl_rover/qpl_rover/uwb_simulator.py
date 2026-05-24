import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import numpy as np

# Must match the -x / -y spawn arguments in sim.launch.py.
# Using odom-frame position + this offset avoids the EKF feedback loop:
# real UWB measures actual distance, not the EKF's estimate of distance.
SPAWN_X = 1.0
SPAWN_Y = 0.5


class UWBSimulator(Node):
    def __init__(self):
        super().__init__('uwb_simulator')

        self.declare_parameter('beacon_x', 0.0)
        self.declare_parameter('beacon_y', 0.0)

        bx = self.get_parameter('beacon_x').get_parameter_value().double_value
        by = self.get_parameter('beacon_y').get_parameter_value().double_value
        self.beacon = np.array([bx, by])

        self.noise_std = 0.05  # 5 cm std — realistic for UWB

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Float32, '/uwb/range', 10)
        self.create_timer(0.1, self.publish_range)

        self.get_logger().info(f"UWB Simulator started. Beacon at ({bx:.2f}, {by:.2f})")

    def publish_range(self):
        # Use odom-frame position (raw wheel encoder integration, unaffected by global EKF
        # drift) plus the known spawn offset to approximate ground truth distance.
        # This prevents the feedback loop where EKF drift corrupts the simulated ranges.
        try:
            tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        pos = np.array([
            tf.transform.translation.x + SPAWN_X,
            tf.transform.translation.y + SPAWN_Y,
        ])
        true_range  = np.linalg.norm(pos - self.beacon)
        noisy_range = true_range + np.random.normal(0.0, self.noise_std)
        msg = Float32()
        msg.data = float(max(0.0, noisy_range))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UWBSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
