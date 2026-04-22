from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rover_pkg: str = get_package_share_directory("qpl_rover")

    twist_mux_params = os.path.join(rover_pkg, "config", "twist_mux.yaml")
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    controller_spawners = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont"],
            ),
        ]
    )

    return LaunchDescription([
        twist_mux,
        controller_spawners
    ])