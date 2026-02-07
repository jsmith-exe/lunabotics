from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():

    package_name = "luna_sim"

    # Path to your saved Gazebo world
    world_path = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "arena.world"
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
    ), launch_arguments={
            "world": world_path,
            "verbose": "true",
        }.items()
    )

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            os.path.join(get_package_share_directory("gazebo_ros"), "worlds", "empty.world"),
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen"
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen"
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'rover'],
                        output='screen')

    return LaunchDescription([
        # Publishes TF base_link -> wheels etc. from robot_description + /joint_states
        rsp,
        gazebo,
        TimerAction(period=5.0, actions=[spawn_entity]),
    ])

