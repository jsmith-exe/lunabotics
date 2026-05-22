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

    drum_lift_bridge = Node(
        package="qpl_rover",
        executable="drum_lift_twist_to_float",
        name="drum_lift_twist_to_float",
        output="screen",
    )

    drum_bridge = Node(
        package="qpl_rover",
        executable="drum_twist_to_float",
        name="drum_twist_to_float",
        output="screen",
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
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["drum_lift_cont"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["drum_cont"],
            ),
        ]
    )

    return LaunchDescription([
        drive_mux,
        drum_lift_bridge,
        drum_bridge,
        controller_spawners
    ])