from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import TimerAction, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition


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
    rover_pkg = get_package_share_directory("qpl_rover")
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).lower()
    use_sim_time_bool = use_sim_time_str == 'true'

    twist_mux_params = os.path.join(rover_pkg, "config", "twist_mux.yaml")

    # Used only for real rover launch because controller_manager is skipped in sim
    controller_params_file = os.path.join(rover_pkg, "config", "my_controllers_real.yaml")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controller_params_file,
            {"use_sim_time": use_sim_time_bool}
        ],
        remappings=[
            ("~/robot_description", "/robot_description")
        ],
        output="screen",
        condition=UnlessCondition(use_sim_time_str),
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            twist_mux_params,
            {"use_sim_time": use_sim_time_bool}
        ],
        remappings=[
            ("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")
        ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[{"use_sim_time": use_sim_time_bool}],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[{"use_sim_time": use_sim_time_bool}],
        output="screen",
    )

    delayed_joint_broad_spawner = TimerAction(
        period=4.0,
        actions=[
            joint_broad_spawner
        ]
    )

    delayed_diff_drive_spawner = TimerAction(
        period=5.0,
        actions=[
            diff_drive_spawner
        ]
    )

    return [
        controller_manager,
        twist_mux,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
    ]