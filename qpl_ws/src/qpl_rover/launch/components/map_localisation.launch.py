from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name = "qpl_rover"
    rover_pkg = get_package_share_directory(package_name)
    ekf_global_params = os.path.join(rover_pkg, "config", "ekf_global_params.yaml")

    # 1. Static Anchor: Where the tag exists in the world
    tag_to_map_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_tag',
        arguments=['0.055', '0.2', '0.3', '1.5708', '0', '1.5708', 'map', 'tag_0']
    )

    # 2. The Observer: Your new Pupil-AprilTags Python node
    apriltag_observer = Node(
        package=package_name,
        executable='apriltag_observer',
        name='apriltag_observer',
        parameters=[{'use_sim_time': True}]
    )

    # 3. Global EKF: Calculates Map -> Odom
    ekf_global_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_global",
                parameters=[ekf_global_params, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "/odometry/global")],
            )
        ],
    )

    # Note: Make sure to include your ekf_local_node here if it's defined elsewhere!

    return LaunchDescription([
        tag_to_map_static,
        apriltag_observer,
        ekf_global_node,
    ])