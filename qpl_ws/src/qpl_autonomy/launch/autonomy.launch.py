from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    config_file = os.path.join(
        get_package_share_directory("qpl_autonomy"),
        "config",
        "fsm_tuning.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),

        Node(
            package="qpl_autonomy",
            executable="autonomy_node",
            name="autonomy",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                config_file,
            ],
        ),
    ])