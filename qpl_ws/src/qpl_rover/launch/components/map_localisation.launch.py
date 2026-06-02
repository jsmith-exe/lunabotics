from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import TimerAction, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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


def opaque_generate_launch_description(context):
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context).lower()
    use_sim_time = use_sim_time_str in ["true", "1", "yes"]

    print(f"use_sim_time: {use_sim_time}")

    package_name = "qpl_rover"
    rover_pkg = get_package_share_directory(package_name)

    if use_sim_time:
        ekf_global_params = os.path.join(
            rover_pkg,
            "config",
            "ekf_global_params.yaml"
        )
    else:
        ekf_global_params = os.path.join(
            rover_pkg,
            "config",
            "ekf_global_params_rover.yaml"
        )

    print(f"Using global EKF params: {ekf_global_params}")

    # 1. Static Anchor: Where the tag exists in the world
    tag_to_map_static = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_tag",
        arguments=[
            "0.055", "0.2", "0.3",
            "0", "0", "0",
            "map", "tag_0"
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2. The Observer: Pupil-AprilTags Python node
    apriltag_observer = Node(
        package=package_name,
        executable="apriltag_observer",
        name="apriltag_observer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 3. Global EKF: Calculates map -> odom
    ekf_global_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_global",
                output="screen",
                parameters=[
                    ekf_global_params,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("odometry/filtered", "/odometry/global")
                ],
            )
        ],
    )

    return [
        tag_to_map_static,
        apriltag_observer,
        ekf_global_node,
    ]