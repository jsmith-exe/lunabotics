from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

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

    # Single source of truth for the tag's map pose (see the comments in it).
    # The observer gets it as parameters; the static TF below is derived from
    # the same values so the two can never disagree.
    tag_pose_params = os.path.join(rover_pkg, "config", "tag_pose.yaml")
    with open(tag_pose_params, "r") as f:
        tag_cfg = yaml.safe_load(f)["apriltag_observer"]["ros__parameters"]
    tag_x, tag_y, tag_z, tag_yaw = (str(v) for v in tag_cfg["tag_map_pose"])
    tag_frame = f"tag_{tag_cfg['tag_id']}"

    # 1. Static Anchor: Where the tag exists in the world (debug/visualisation)
    tag_to_map_static = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_tag",
        arguments=[
            tag_x, tag_y, tag_z,
            tag_yaw, "0", "0",  # yaw pitch roll
            "map", tag_frame
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2. The Observer: Pupil-AprilTags Python node
    apriltag_observer = Node(
        package=package_name,
        executable="apriltag_observer",
        name="apriltag_observer",
        output="screen",
        parameters=[
            tag_pose_params,
            {"use_sim_time": use_sim_time},
        ],
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
