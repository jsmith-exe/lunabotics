import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = "qpl_rover"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), "description", "rover.urdf.xacro"]
            ),
            " ",
            "use_ros2_control:=true",
            " ",
            "sim_mode:=false",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)}

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "twist_mux.yaml"
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": False}],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": False}],
        output="screen",
    )

    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "my_controllers.yaml"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output="screen",
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen",
    )

    delayed_controller_manager = TimerAction(
        period=2.0,
        actions=[controller_manager]
    )

    delayed_joint_broad_spawner = TimerAction(
        period=4.0,
        actions=[joint_broad_spawner]
    )

    delayed_diff_drive_spawner = TimerAction(
        period=5.0,
        actions=[diff_drive_spawner]
    )

    return LaunchDescription([
        robot_state_publisher,
        twist_mux,
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
    ])