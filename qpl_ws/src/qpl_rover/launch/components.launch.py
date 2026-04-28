"""
Responsible for packaging up the component launch files into a single launch file.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

rover_pkg: str = get_package_share_directory("qpl_rover")

def get_component_python_launch(name: str) -> PythonLaunchDescriptionSource:
    return PythonLaunchDescriptionSource(
        os.path.join(rover_pkg, "launch", f"{name}.launch.py")
    )

def generate_launch_description():
    controllers = IncludeLaunchDescription(get_component_python_launch("controllers"))
    odom_localisation = IncludeLaunchDescription(get_component_python_launch("odom_localisation"))
    map_localisation = IncludeLaunchDescription(get_component_python_launch("map_localisation"))

    return LaunchDescription([
        controllers,
        odom_localisation,
        map_localisation,
    ])