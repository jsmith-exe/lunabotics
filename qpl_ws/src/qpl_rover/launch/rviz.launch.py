from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    pkg = get_package_share_directory("qpl_rover")

    rviz_config = os.path.join(pkg, "config", "cost_map.rviz")

    # RViz (loads your config)
    rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )

    return LaunchDescription([
        rviz,
        
        
    ])