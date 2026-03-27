from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

config_name = "cost_map"

def generate_launch_description():
    basestation_pkg_path = get_package_share_directory("basestation")

    rviz_config = os.path.join(basestation_pkg_path, "rviz", f"{config_name}.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        rviz,
    ])