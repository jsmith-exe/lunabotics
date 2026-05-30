"""
Responsible for packaging up the component launch files into a single launch file.
"""
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

rover_pkg: str = get_package_share_directory("qpl_rover")

def generate_launch_description():
    use_sim_time_parameter = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Enable if using simulation'
    )

    return LaunchDescription([
        use_sim_time_parameter,
        OpaqueFunction(function=opaque_generate_launch_description),
    ])

def opaque_generate_launch_description(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    controllers = IncludeLaunchDescription(get_component_python_launch("controllers"))
    odom_localisation = IncludeLaunchDescription(get_component_python_launch("odom_localisation"))
    map_localisation = IncludeLaunchDescription(get_component_python_launch("map_localisation"))

    return [
        SetParameter(name='use_sim_time', value='false'),
        controllers,
        odom_localisation,
        map_localisation,
    ]

def get_component_python_launch(name: str) -> PythonLaunchDescriptionSource:
    return PythonLaunchDescriptionSource(
        os.path.join(rover_pkg, "launch", f"{name}.launch.py")
    )