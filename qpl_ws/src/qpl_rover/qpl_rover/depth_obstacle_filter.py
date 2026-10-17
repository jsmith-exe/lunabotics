"""
Depth image -> Nav2 obstacle clouds.

Nav2's own height filter measures z in the costmap's global frame, which assumes
the floor is at z = 0 everywhere. Nothing measures the rover's z, so on the
terrain world a crest ahead of a rover sitting in a dip crosses the threshold
and is marked, and floor hits are dropped before raytracing so nothing ever
clears. This node does the height reasoning instead and hands Nav2 two
pre-classified, flattened clouds:

  <camera>/obstacles  points that are obstacles      (Nav2: marking only)
  <camera>/clearing   every valid return, incl. floor (Nav2: clearing only)

Heights are measured in a gravity-levelled frame centred on the rover: points go
into `level_frame` (odom, whose roll/pitch come from the IMU via the EKF), then
the rover's own ground point (the base_footprint origin) is subtracted. The arena
floor is bounded (+-7 cm on the terrain world), so in a level frame the floor
anywhere stays within ~0.14 m of the floor under the rover, even when the rover
is parked on a slope. A body-fixed frame would not: a 6 deg tilt puts floor 4 m
away 0.42 m off.

The obstacle threshold grows with range to absorb IMU tilt error (a 2 deg error
moves a point 4 m away by 0.14 m):

  min_height(r) = min_obstacle_height + r * height_slope_per_m

Walls (0.5 m in the sim) stay well above that at every range.

Both outputs are in `level_frame` with z set to the sensor's z, so Nav2's 3D
range checks act as 2D ones and its height band (set wide) never rejects them.
Nav2 must be given `sensor_frame` for these sources, since the cloud's own frame
(odom) is not where the rays start.

Input is the depth image, not the driver's point cloud: the sim clouds are
10-13 MB a frame and were dropped in transport for seconds at a time (Nav2 then
reuses the last cloud, from wherever the rover was). The image is ~8x smaller and
is deprojected here at the decimated resolution only.
"""
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
import tf2_ros


def quaternion_matrix(q):
    """3x3 rotation matrix for a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_parts(tf):
    t = tf.transform.translation
    return quaternion_matrix(tf.transform.rotation), np.array([t.x, t.y, t.z])


def deproject(image, info, stride):
    """(N, 3) float array of optical-frame points from every `stride`th depth pixel."""
    if image.encoding == '16UC1':
        depth = np.frombuffer(image.data, dtype=np.uint16).reshape(image.height, -1)[:, :image.width]
        scale = 0.001  # mm
    elif image.encoding == '32FC1':
        depth = np.frombuffer(image.data, dtype=np.float32).reshape(image.height, -1)[:, :image.width]
        scale = 1.0
    else:
        raise ValueError(f'unsupported depth encoding {image.encoding}')

    # camera_info can be for a different resolution than the image (RealSense decimation).
    sx = image.width / info.width
    sy = image.height / info.height
    fx, fy = info.k[0] * sx, info.k[4] * sy
    cx, cy = info.k[2] * sx, info.k[5] * sy

    v, u = np.mgrid[0:image.height:stride, 0:image.width:stride]
    z = depth[::stride, ::stride].astype(np.float64) * scale
    valid = np.isfinite(z) & (z > 0)
    z, u, v = z[valid], u[valid], v[valid]
    return np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)


XYZ_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


def make_cloud(header, xyz):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(xyz)
    msg.fields = XYZ_FIELDS
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(xyz)
    msg.is_dense = True
    msg.data = xyz.astype(np.float32).tobytes()
    return msg


class DepthObstacleFilter(Node):
    def __init__(self):
        super().__init__('depth_obstacle_filter')

        self.depth_topic = self.declare_parameter('depth_topic', '/depth_camera_front/depth/image_raw').value
        self.info_topic = self.declare_parameter('info_topic', '/depth_camera_front/depth/camera_info').value
        self.output_namespace = self.declare_parameter('output_namespace', '/depth_camera_front').value
        self.level_frame = self.declare_parameter('level_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value

        # Decimation and rate: the Orbbec runs 640x480 at 30 Hz, and the RealSense is already
        # decimated 4x on the rover but not in the sim, so decimate to a width, not by a factor.
        self.target_width = int(self.declare_parameter('target_width', 160).value)
        self.max_rate = float(self.declare_parameter('max_rate', 10.0).value)

        # Ranges are horizontal distance from the sensor.
        self.min_range = self.declare_parameter('min_range', 0.30).value
        self.obstacle_max_range = self.declare_parameter('obstacle_max_range', 4.0).value
        self.clearing_max_range = self.declare_parameter('clearing_max_range', 5.0).value

        # Height above the floor under the rover.
        self.min_obstacle_height = self.declare_parameter('min_obstacle_height', 0.16).value
        self.height_slope_per_m = self.declare_parameter('height_slope_per_m', 0.035).value
        self.max_obstacle_height = self.declare_parameter('max_obstacle_height', 1.0).value

        # Points inside this base_frame box are the rover itself.
        self.self_filter_x = self.declare_parameter('self_filter_x', 0.65).value
        self.self_filter_y = self.declare_parameter('self_filter_y', 0.56).value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.obstacle_pub = self.create_publisher(
            PointCloud2, f'{self.output_namespace}/obstacles', qos_profile_sensor_data)
        self.clearing_pub = self.create_publisher(
            PointCloud2, f'{self.output_namespace}/clearing', qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)

        self.info = None
        self.last_stamp = None
        self.get_logger().info(
            f'{self.depth_topic} -> {self.output_namespace}/{{obstacles,clearing}} in {self.level_frame}')

    def on_info(self, msg):
        self.info = msg

    def lookup(self, target, source, stamp):
        """Transform at `stamp`; if TF hasn't caught up yet, the latest one if it's within 0.1 s."""
        try:
            return self.tf_buffer.lookup_transform(target, source, stamp, Duration(seconds=0.1))
        except tf2_ros.ExtrapolationException:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
            if abs((Time.from_msg(tf.header.stamp) - stamp).nanoseconds) > 1e8:
                raise
            return tf

    def on_depth(self, msg):
        if self.info is None:
            self.get_logger().warn(f'No camera_info on {self.info_topic} yet', throttle_duration_sec=5.0)
            return

        stamp = Time.from_msg(msg.header.stamp)
        # A stamp going backwards (sim reset) resets the throttle.
        if self.last_stamp is not None and 0 <= (stamp - self.last_stamp).nanoseconds < 1e9 / self.max_rate:
            return
        self.last_stamp = stamp

        try:
            sensor_tf = self.lookup(self.level_frame, msg.header.frame_id, stamp)
            base_tf = self.lookup(self.level_frame, self.base_frame, stamp)
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return

        sensor_rot, sensor_pos = transform_parts(sensor_tf)
        base_rot, base_pos = transform_parts(base_tf)

        stride = max(1, msg.width // self.target_width)
        points = deproject(msg, self.info, stride) @ sensor_rot.T + sensor_pos

        # Drop the rover itself (box in base_frame).
        in_base = (points - base_pos) @ base_rot
        rover = (np.abs(in_base[:, 0]) < self.self_filter_x) & (np.abs(in_base[:, 1]) < self.self_filter_y)
        points = points[~rover]

        rng = np.hypot(points[:, 0] - sensor_pos[0], points[:, 1] - sensor_pos[1])
        height = points[:, 2] - base_pos[2]

        clearing = (rng >= self.min_range) & (rng <= self.clearing_max_range) & (height <= self.max_obstacle_height)
        obstacle = (
            clearing
            & (rng <= self.obstacle_max_range)
            & (height >= self.min_obstacle_height + rng * self.height_slope_per_m)
        )

        flat = points.copy()
        flat[:, 2] = sensor_pos[2]

        header = msg.header
        header.frame_id = self.level_frame
        self.obstacle_pub.publish(make_cloud(header, flat[obstacle]))
        self.clearing_pub.publish(make_cloud(header, flat[clearing]))


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
