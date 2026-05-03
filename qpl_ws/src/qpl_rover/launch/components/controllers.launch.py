from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rover_pkg: str = get_package_share_directory("qpl_rover")

    drive_mux_params = os.path.join(rover_pkg, "config", "drive_mux.yaml")
    drive_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="drive_mux",
        output="screen",
        parameters=[drive_mux_params, {"use_sim_time": True}],
        remappings=[("cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    drum_mux_params = os.path.join(rover_pkg, "config", "drum_mux.yaml")
    drum_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="drum_mux",
        output="screen",
        parameters=[drum_mux_params, {"use_sim_time": True}],
        remappings=[("cmd_vel_out", "/drum_cont/cmd_vel_unstamped")],
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
        drive_mux,
        drum_mux,
        controller_spawners
    ])