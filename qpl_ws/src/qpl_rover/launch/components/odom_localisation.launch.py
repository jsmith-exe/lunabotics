from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import TimerAction, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def opaque_generate_launch_description(context):
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context).lower()
    use_sim_time = use_sim_time_str in ["true", "1", "yes"]

    print(f"use_sim_time: {use_sim_time}")

    rover_pkg = get_package_share_directory("qpl_rover")

    if use_sim_time:
        ekf_local_params = os.path.join(
            rover_pkg,
            "config",
            "ekf_local_params.yaml"
        )
    else:
        ekf_local_params = os.path.join(
            rover_pkg,
            "config",
            "ekf_local_params_rover.yaml"
        )

    print(f"Using EKF params: {ekf_local_params}")

    ekf_local_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_local",
                output="screen",
                parameters=[
                    ekf_local_params,
                    {"use_sim_time": use_sim_time},
                ],
            )
        ],
    )

    return [
        ekf_local_node,
    ]


def generate_launch_description():
    use_sim_time_parameter = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Enable if using simulation"
    )

    return LaunchDescription([
        use_sim_time_parameter,
        OpaqueFunction(function=opaque_generate_launch_description),
    ])