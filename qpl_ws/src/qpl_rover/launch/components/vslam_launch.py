"""
RGB-D visual odometry for dead reckoning.

Runs rtabmap_odom/rgbd_odometry against the FRONT depth camera and publishes a
body-velocity estimate on /vo/odom for the local EKF to fuse as odom1. It does
not publish TF and it does not map.

Usage:
    ros2 launch qpl_rover vslam_launch.py use_sim_time:=true
    qpl_vslam                                  # alias, hardware defaults
    qpl_sim use_vslam:=true                    # integrated with the sim stack
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# The camera the RGB-D pair comes from, per environment. Both entries must name
# a colour image and a depth image that are mutually registered: same
# resolution, same intrinsics, same optical frame. Mixing streams from two
# different sensors will sync but produce garbage odometry.
SIM_TOPICS = {
    # Both from the single 848x480 Gazebo depth sensor.
    "rgb": "/depth_camera_front_depth/image_raw",
    "info": "/depth_camera_front_depth/camera_info",
    "depth": "/depth_camera_front_depth/depth/image_raw",
}

ROVER_TOPICS = {
    # RealSense colour, plus depth registered into the colour frame by the
    # firmware. Requires align_depth.enable in camera_realsense.launch.py.
    "rgb": "/depth_camera_front/color/image_raw",
    "info": "/depth_camera_front/color/camera_info",
    "depth": "/depth_camera_front/aligned_depth_to_color/image_raw",
}


def opaque_generate_launch_description(context):
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context).lower()
    use_sim_time = use_sim_time_str in ["true", "1", "yes"]

    rover_pkg = get_package_share_directory("qpl_rover")

    if use_sim_time:
        params_file = os.path.join(rover_pkg, "config", "vslam_odom_params.yaml")
        topics = SIM_TOPICS
    else:
        params_file = os.path.join(rover_pkg, "config", "vslam_odom_params_rover.yaml")
        topics = ROVER_TOPICS

    print(f"[vslam] use_sim_time: {use_sim_time}")
    print(f"[vslam] params: {params_file}")
    print(f"[vslam] rgb:   {topics['rgb']}")
    print(f"[vslam] depth: {topics['depth']}")
    print(f"[vslam] info:  {topics['info']}")
    print("[vslam] publishing odometry on /vo/odom (no TF)")

    visual_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="visual_odometry",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("rgb/image", topics["rgb"]),
            ("rgb/camera_info", topics["info"]),
            ("depth/image", topics["depth"]),
            # The EKF configs' odom1 must match this exactly.
            ("odom", "/vo/odom"),
        ],
        # VO is best-effort: if it dies the filter coasts on wheel odom + gyro.
        respawn=True,
        respawn_delay=4.0,
    )

    return [visual_odometry]


def generate_launch_description():
    use_sim_time_parameter = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Enable if using simulation",
    )

    return LaunchDescription([
        use_sim_time_parameter,
        OpaqueFunction(function=opaque_generate_launch_description),
    ])
