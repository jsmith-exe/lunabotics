from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

WORLD = "arena.world"

def generate_launch_description():
    sim_package_path = get_package_share_directory("luna_sim")
    world_path = os.path.join(sim_package_path, "worlds", WORLD)
    gazebo_params_path = os.path.join(sim_package_path, "config", "gazebo_params.yaml")
    rsp_launch_path = os.path.join(sim_package_path, 'launch', 'rsp.launch.py')
    gazebo_launch_path = os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
    rover_xacro_path = os.path.join(sim_package_path, "description", "rover.urdf.xacro")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={"world": world_path, 'extra_gazebo_args': '--ros-args --params-file' + gazebo_params_path}.items(),
    )

    generate_urdf = ExecuteProcess(
        cmd=["bash", "-c", f"xacro {rover_xacro_path} > /tmp/rover.urdf"],
        output="screen",
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "rover"],
        output="screen",
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

