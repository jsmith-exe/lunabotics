from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():
    package_name = "qpl_rover"

    ekf_local_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "ekf_local_params.yaml",
    )

    # Delay EKF until after controllers are up (period=4.0) so the robot exists
    # and /diff_cont/odom + /imu/data are already publishing before EKF initialises.
    # Starting EKF too early causes a bad initial state that never recovers cleanly.
    ekf_local_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_local",
                output="screen",
                parameters=[ekf_local_params, {"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription([
        ekf_local_node,
    ])