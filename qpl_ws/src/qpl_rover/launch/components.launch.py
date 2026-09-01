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

    use_vslam_parameter = DeclareLaunchArgument(
        'use_vslam',
        default_value='false',
        description='Run RGB-D visual odometry and fuse it into the local EKF as odom1.'
    )

    return LaunchDescription([
        use_sim_time_parameter,
        use_vslam_parameter,
        OpaqueFunction(function=opaque_generate_launch_description),
    ])

def opaque_generate_launch_description(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_vslam = LaunchConfiguration('use_vslam').perform(context).lower() in ["true", "1", "yes"]

    launch_arguments = {"use_sim_time": use_sim_time}.items()
    controllers = IncludeLaunchDescription(get_component_python_launch("controllers"), launch_arguments=launch_arguments)
    odom_localisation = IncludeLaunchDescription(get_component_python_launch("odom_localisation"), launch_arguments=launch_arguments)
    map_localisation = IncludeLaunchDescription(get_component_python_launch("map_localisation"), launch_arguments=launch_arguments)

    components = [
        controllers,
        odom_localisation,
        map_localisation,
    ]

    if use_vslam:
        # Named vslam_launch.py rather than vslam.launch.py so the existing
        # `qpl_vslam` alias in process/functions/executables.sh keeps working.
        components.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rover_pkg, "launch", "vslam_launch.py")
                ),
                launch_arguments=launch_arguments,
            )
        )

    return components

def get_component_python_launch(name: str) -> PythonLaunchDescriptionSource:
    return PythonLaunchDescriptionSource(
        os.path.join(rover_pkg, "launch", f"{name}.launch.py")
    )