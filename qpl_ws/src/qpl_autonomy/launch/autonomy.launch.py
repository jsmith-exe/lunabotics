from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

MODE_EXECUTABLES = {
    "odom": "autonomy_node",
    "localised": "localised_autonomy_node",
    "timed": "timed_autonomy_node",
}

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    mode = LaunchConfiguration("mode")

    config_file = os.path.join(
        get_package_share_directory("qpl_autonomy"),
        "config",
        "fsm_tuning.yaml", # this will pull in the FSM tuning params
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="odom",
            choices=list(MODE_EXECUTABLES),
            description="How drive stages are measured: odom (drivetrain odometry), localised (map -> base_footprint) or timed",
        ),

        Node(
            package="qpl_autonomy",
            executable=PythonExpression([repr(MODE_EXECUTABLES), "['", mode, "']"]),
            name="autonomy",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                config_file,
            ],
        ),
    ])
