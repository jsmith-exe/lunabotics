from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch.actions import ExecuteProcess



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

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), "config", "gazebo_params.yaml")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_path,
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file,
        }.items(),
    )



    pkg_share = get_package_share_directory("luna_sim")
    xacro_file = os.path.join(pkg_share, "description", "rover.urdf.xacro")

    generate_urdf = ExecuteProcess(
        cmd=["bash", "-c", f"xacro {xacro_file} > /tmp/rover.urdf"],
        output="screen",
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "rover"],
        output="screen",
    )

    controllers_yaml = os.path.join(
        get_package_share_directory("luna_sim"),
        "config",
        "my_controllers.yaml",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/cmd_vel", "/diff_cont/cmd_vel_unstamped"],
        output="screen",
    )


    return LaunchDescription([
        rsp,
        gazebo,
        generate_urdf,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        cmd_vel_relay,
        
    ])

