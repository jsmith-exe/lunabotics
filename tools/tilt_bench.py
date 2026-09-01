"""Synthetic tilt bench: publish a stationary IMU at a known roll/pitch and
check what ekf_local recovers. No hardware, no Gazebo."""
import math, sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

TRUE_ROLL_DEG, TRUE_PITCH_DEG = float(sys.argv[1]), float(sys.argv[2])
G = 9.80665

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

def quat_to_rpy(x, y, z, w):
    r = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    s = max(-1.0, min(1.0, 2*(w*y - z*x)))
    p = math.asin(s)
    yw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return r, p, yw

class Bench(Node):
    def __init__(self):
        super().__init__("tilt_bench")
        self.pub = self.create_publisher(Imu, "/camera/camera/imu", 10)
        self.odom_pub = self.create_publisher(Odometry, "/diff_cont/odom", 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, 10)
        self.create_timer(1/100.0, self.tick)
        self.last = None

        R_world_body = rpy(math.radians(TRUE_ROLL_DEG), math.radians(TRUE_PITCH_DEG), 0.0)
        # Accelerometer at rest reads the reaction to gravity: +g along world up.
        a_body = R_world_body.T @ np.array([0.0, 0.0, G])
        # base_footprint -> camera_imu_optical_frame is RPY(-90, 0, -90).
        R_bf_opt = rpy(-math.pi/2, 0.0, -math.pi/2)
        self.a_opt = R_bf_opt.T @ a_body

    def tick(self):
        now = self.get_clock().now().to_msg()
        m = Imu()
        m.header.stamp = now
        m.header.frame_id = "camera_imu_optical_frame"
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = self.a_opt
        m.angular_velocity.x = m.angular_velocity.y = m.angular_velocity.z = 0.0
        m.orientation_covariance[0] = -1.0          # as the RealSense driver does
        m.angular_velocity_covariance[0] = m.angular_velocity_covariance[4] = m.angular_velocity_covariance[8] = 0.001
        m.linear_acceleration_covariance[0] = m.linear_acceleration_covariance[4] = m.linear_acceleration_covariance[8] = 0.01
        self.pub.publish(m)

        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = "odom"
        o.child_frame_id = "base_footprint"
        o.twist.covariance[0], o.twist.covariance[7], o.twist.covariance[35] = 0.01, 0.05, 0.05
        self.odom_pub.publish(o)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.last = (quat_to_rpy(q.x, q.y, q.z, q.w), msg.pose.pose.position.z)

def main():
    rclpy.init()
    n = Bench()
    end = n.get_clock().now().nanoseconds + int(25e9)
    while rclpy.ok() and n.get_clock().now().nanoseconds < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.last is None:
        print("NO OUTPUT on /odometry/filtered"); return 1
    (r, p, y), z = n.last
    print(f"  commanded tilt : roll {TRUE_ROLL_DEG:+7.2f}   pitch {TRUE_PITCH_DEG:+7.2f}")
    print(f"  ekf_local says : roll {math.degrees(r):+7.2f}   pitch {math.degrees(p):+7.2f}   yaw {math.degrees(y):+7.2f}")
    print(f"  z after 25 s   : {z:+.5f} m")
    ok = abs(math.degrees(r) - TRUE_ROLL_DEG) < 1.0 and abs(math.degrees(p) - TRUE_PITCH_DEG) < 1.0
    print("  ->", "PASS" if ok else "FAIL")
    return 0 if ok else 1

sys.exit(main())
