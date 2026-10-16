"""Full autonomy in the sim with one command (qpl_auto_sim).

Starts the sim, Nav2 and the autonomy node, then AUTO begins by itself once Nav2 is up
and the rover has its first AprilTag fix. Stop it with qpl_stop.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rover_pkg = get_package_share_directory("qpl_rover")
    config_file = os.path.join(
        get_package_share_directory("qpl_autonomy"), "config", "fsm_tuning.yaml")

    # In the start zone, facing the AprilTag on the south wall from far enough back to
    # detect it, so the rover is localised without driving first.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rover_pkg, "launch", "sim.launch.py")),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "spawn_x": "3.40",
            "spawn_y": "1.95",
            "spawn_yaw": "-1.5708",
        }.items(),
    )

    # Delayed so the rover, controllers and EKFs are up before Nav2 looks for TF.
    nav = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rover_pkg, "launch", "navigation_launch.py")),
                # params_file must be explicit: gazebo_ros's gzserver launch declares its own
                # params_file (empty), which would otherwise replace Nav2's default.
                launch_arguments={
                    "use_sim_time": "true",
                    "params_file": os.path.join(rover_pkg, "config", "nav_params.yaml"),
                }.items(),
            )
        ],
    )

    autonomy = Node(
        package="qpl_autonomy",
        executable="autonomy_node",
        name="autonomy",
        output="screen",
        parameters=[
            config_file,
            {"use_sim_time": True, "autonomy.auto_start": True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo without its GUI",
        ),
        sim,
        nav,
        autonomy,
    ])
