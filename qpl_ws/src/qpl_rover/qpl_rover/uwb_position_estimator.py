import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from scipy.spatial import ConvexHull
import numpy as np
from scipy.optimize import least_squares

WINDOW_SIZE      = 15    # more samples = better geometry
MIN_HULL_AREA    = 0.15  # m² — minimum convex hull area of window positions
MAX_COST         = 0.8   # tighter fit requirement
MAX_OFFSET_JUMP  = 0.5   # m — reject if offset jumps more than this

ARENA_X = (-0.3, 4.7)
ARENA_Y = (-0.3, 8.2)

BASE_COV    = 0.1   # m² baseline position variance
COV_SCALE   = 5.0   # multiplier on solve cost to inflate covariance for poor solves


class UWBPositionEstimator(Node):
    def __init__(self):
        super().__init__('uwb_position_estimator')

        self.declare_parameter('beacon_x', 0.0)
        self.declare_parameter('beacon_y', 0.0)

        bx = self.get_parameter('beacon_x').get_parameter_value().double_value
        by = self.get_parameter('beacon_y').get_parameter_value().double_value
        self.beacon = np.array([bx, by])

        self.window = []   # list of (odom_pos, range)

        # Only ever updated from AprilTag detections — never from UWB solves.
        # Prevents error accumulation across successive solves.
        self.apriltag_offset = None

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Float32,                   '/uwb/range',     self.range_cb,    10)
        self.create_subscription(PoseWithCovarianceStamped, '/apriltag/pose', self.apriltag_cb, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/uwb/pose', 10)

        self.get_logger().info(f"UWB Position Estimator started. Beacon at ({bx:.2f}, {by:.2f})")

    def get_odom_pos(self):
        try:
            tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            return np.array([tf.transform.translation.x, tf.transform.translation.y])
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def apriltag_cb(self, msg):
        odom_pos = self.get_odom_pos()
        if odom_pos is None:
            return
        tag_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ])
        self.apriltag_offset = tag_pos - odom_pos

    def range_cb(self, msg):
        odom_pos = self.get_odom_pos()
        if odom_pos is None:
            return

        self.window.append((odom_pos, msg.data))
        if len(self.window) > WINDOW_SIZE:
            self.window.pop(0)

        if len(self.window) < WINDOW_SIZE:
            return

        if not self.window_has_geometry():
            return

        result = self.solve()
        if result is not None:
            pos, cost = result
            self.publish_pose(pos, cost)

    def window_has_geometry(self):
        positions = np.array([e[0] for e in self.window])

        # Need at least 3 unique points for a convex hull
        unique = np.unique(positions, axis=0)
        if len(unique) < 3:
            return False

        try:
            hull = ConvexHull(positions)
            return hull.volume >= MIN_HULL_AREA  # volume = area in 2D
        except Exception:
            return False

    def solve(self):
        if self.apriltag_offset is None:
            return None

        odom_positions = np.array([e[0] for e in self.window])
        ranges         = np.array([e[1] for e in self.window])

        def residuals(offset):
            shifted   = odom_positions + offset
            predicted = np.linalg.norm(shifted - self.beacon, axis=1)
            return predicted - ranges

        result = least_squares(residuals, x0=self.apriltag_offset, method='lm')

        if result.cost > MAX_COST:
            self.get_logger().debug(f"UWB rejected — cost {result.cost:.3f}")
            return None

        offset_shift = np.linalg.norm(result.x - self.apriltag_offset)
        if offset_shift > MAX_OFFSET_JUMP:
            self.get_logger().debug(f"UWB rejected — offset jumped {offset_shift:.2f} m")
            return None

        pos = odom_positions[-1] + result.x

        if not (ARENA_X[0] <= pos[0] <= ARENA_X[1] and ARENA_Y[0] <= pos[1] <= ARENA_Y[1]):
            self.get_logger().debug(f"UWB rejected — outside arena ({pos[0]:.2f}, {pos[1]:.2f})")
            return None

        return pos, result.cost

    def publish_pose(self, pos, cost):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = pos[0]
        msg.pose.pose.position.y = pos[1]
        msg.pose.pose.orientation.w = 1.0

        # Scale covariance with solve cost so the EKF trusts poor solves less.
        # cost=0 → cov=BASE_COV, cost=MAX_COST → cov≈BASE_COV*(1+COV_SCALE)
        variance = BASE_COV * (1.0 + COV_SCALE * cost)
        cov = np.zeros(36)
        cov[0]  = variance
        cov[7]  = variance
        cov[35] = 9999.0
        msg.pose.covariance = cov.tolist()

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UWBPositionEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
