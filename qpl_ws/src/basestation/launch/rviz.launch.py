from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration




def generate_launch_description():
    basestation_pkg_path = get_package_share_directory("basestation")

    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true."
    )

    rviz_config = os.path.join(basestation_pkg_path, "rviz", "default.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{
            "use_sim_time": use_sim_time,
            # Decode the teleop streams in software. The ffmpeg plugin otherwise picks the
            # NVIDIA h264_cuvid decoder, which buffers several frames before showing any (a lot
            # of lag at low frame rates, e.g. a slow Gazebo) and has shown CUDA errors that
            # turn the picture green. The software decoder outputs each frame immediately.
            "depth_camera_front.color.teleop_stream.ffmpeg.decoders.h264": "h264",
            "depth_camera_rear.color.teleop_stream.ffmpeg.decoders.h264": "h264",
        }],
        output="screen",
    )

    # Arena zone + mission-waypoint overlay (/zone_overlay MarkerArray) to help
    # the teleoperator see the zones and the autonomy targets in the map frame.
    zone_overlay = Node(
        package="basestation",
        executable="zone_overlay",
        name="zone_overlay",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz,
        zone_overlay,
    ])