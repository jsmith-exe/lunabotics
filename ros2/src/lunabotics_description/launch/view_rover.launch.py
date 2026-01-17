from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("lunabotics_description")
    urdf_path = os.path.join(pkg, "urdf", "basic_rover.urdf")
    rviz_cfg  = os.path.join(pkg, "rviz", "rover.rviz")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen",
        ),
    ])
