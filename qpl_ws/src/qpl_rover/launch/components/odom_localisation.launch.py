from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rover_pkg: str = get_package_share_directory('qpl_rover')

    # Delay EKF until after controllers are up (period=4.0) so the robot exists
    # and /diff_cont/odom + /imu/data are already publishing before EKF initialises.
    # Starting EKF too early causes a bad initial state that never recovers cleanly.
    ekf_local_params = os.path.join(rover_pkg, "config", "ekf_local_params.yaml")
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