"""
RGB-D visual odometry for dead reckoning.

Runs rtabmap_odom/rgbd_odometry against the FRONT depth camera and publishes a
body-velocity estimate on /vo/odom for the local EKF to fuse as odom1. It does
not publish TF and it does not map — Nav2's voxel layer replaced the old
RTAB-Map perception node in 066ef28, and only the odometry half is wanted here.

Why this exists again: an earlier version was deleted in efad12f "Remove vslam"
having never worked. Three things were wrong and all three are fixed here:

  1. The EKF subscribed to /rtabmap/visual_odom, but the node published
     /rtabmap/odom (namespace + default topic name). Nothing ever published
     the topic the filter was listening to. The output is now remapped to an
     explicit /vo/odom, stated in one place, matching what the EKF configs say.

  2. The sim remappings pointed at /depth_camera_front/depth/image_raw. That
     path stopped existing when the front camera was split into two co-located
     Gazebo sensors: a 1280x800 colour camera (camera_name depth_camera_front)
     and an 848x480 depth camera (camera_name depth_camera_front_depth). Those
     two have different resolutions and different camera_info, so they are not
     a usable RGB-D pair. The depth sensor publishes its own colour image at
     its own resolution, sharing intrinsics, stamps and optical frame — that is
     the registered pair this node now uses. No xacro change needed.

  3. It fused absolute pose as well as velocity. See the odom1 block in the EKF
     configs: twist only.

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
    # firmware. Requires align_depth.enable on the camera node — see
    # camera_realsense.launch.py.
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
            # The fix for bug 1. The EKF configs' odom1 must match this exactly.
            ("odom", "/vo/odom"),
        ],
        # VO is a best-effort input the EKF can live without; if it dies, the
        # filter coasts on wheel odom + gyro rather than the stack going down.
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
