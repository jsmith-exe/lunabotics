import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import matplotlib.pyplot as plt

rclpy.init()
node = Node('pose_plotter')

plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(0, 4.4)
ax.set_ylim(0, 7.9)
ax.set_aspect('equal')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Rover Pose')
ax.grid(True)

dot, = ax.plot([], [], 'ro', markersize=10)


def cb(msg):
    y = -msg.x
    x = msg.y
    dot.set_xdata([x])
    dot.set_ydata([y])
    fig.canvas.draw_idle()
    fig.canvas.flush_events()


node.create_subscription(Pose2D, '/apriltag_pose_2d', cb, 10)
rclpy.spin(node)