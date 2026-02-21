from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz_pkg_path = get_package_share_directory("luna_rviz")
    rviz_config_path = os.path.join(rviz_pkg_path, "rviz", "basestation_config.rviz")

    # RViz (loads your config)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    return LaunchDescription([
        rviz,
    ])