#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ImuOpticalToStandard(Node):
    def __init__(self):
        super().__init__('imu_optical_to_standard')
        
        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(Imu, '/camera/camera/imu', self.cb, input_qos)
        self.pub = self.create_publisher(Imu, '/camera/camera/imu_standard', output_qos)

    def cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = 'camera_imu_frame'  # standard-convention frame

        # Rotation: x_std = z_opt, y_std = -x_opt, z_std = -y_opt
        av = msg.angular_velocity
        out.angular_velocity.x = av.z
        out.angular_velocity.y = -av.x
        out.angular_velocity.z = -av.y

        la = msg.linear_acceleration
        out.linear_acceleration.x = la.z
        out.linear_acceleration.y = -la.x
        out.linear_acceleration.z = -la.y

        # Orientation unused (use_mag: False path doesn't trust it anyway)
        out.orientation_covariance[0] = -1.0

        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ImuOpticalToStandard())

if __name__ == '__main__':
    main()