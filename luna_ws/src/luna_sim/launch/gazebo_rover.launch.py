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
        "cones.world"
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("gazebo_ros"),
            "launch",
            "gazebo.launch.py",
        )
    ),
    launch_arguments={"verbose": "true"}.items(),
    )

    pkg_share = get_package_share_directory("luna_sim")
    xacro_file = os.path.join(pkg_share, "description", "rover.urdf.xacro")

    generate_urdf = ExecuteProcess(
        cmd=["bash", "-c", f"xacro {xacro_file} > /tmp/rover.urdf"],
        output="screen",
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
    spawn_entity = Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    arguments=["-file", "/tmp/rover.urdf", "-entity", "rover"],
    output="screen",
    )



    controllers_yaml = os.path.join(
        get_package_share_directory("luna_sim"),
        "config",
        "ros2_control_controllers.yaml",
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped"],
        output="screen",
    )

    return LaunchDescription([
        rsp,
        gazebo,

        generate_urdf,

        TimerAction(period=6.0, actions=[spawn_entity]),
        TimerAction(period=12.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=13.0, actions=[diff_drive_spawner]),
        cmd_vel_relay,
    ])

