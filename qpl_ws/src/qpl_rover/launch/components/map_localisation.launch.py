from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():
    package_name = "qpl_rover"

    apriltag_config = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "apriltag.yaml",
    )

    apriltag_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag_node",
                output="screen",
                remappings=[
                    ("image_rect", "/depth_camera_rear/image_raw"),
                    ("camera_info", "/depth_camera_rear/camera_info"),
                ],
                parameters=[apriltag_config, {"use_sim_time": True}],
            )
        ],
    )

    apriltag_map_odom = TimerAction(
        period=8.0,
        actions=[
            Node(
                package=package_name,
                executable="apriltag_map_odom_3d",
                name="apriltag_map_odom_3d",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    ekf_global_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "ekf_global_params.yaml",
    )

    # Delay EKF until after controllers are up (period=4.0) so the robot exists
    # and /diff_cont/odom + /imu/data are already publishing before EKF initialises.
    # Starting EKF too early causes a bad initial state that never recovers cleanly.
    # TODO WIP
    ekf_global_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_global",
                output="screen",
                parameters=[ekf_global_params, {"use_sim_time": True}],
                remappings=[
                    ("odometry/filtered", "/odometry/global"),
                ],
            )
        ],
    )

    return LaunchDescription([
        apriltag_node,
        apriltag_map_odom
    ])