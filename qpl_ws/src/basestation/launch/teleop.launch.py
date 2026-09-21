"""Teleop-focused RViz bringup.

Two rviz2 windows placed side by side on a 1920x1080 screen: the driving window
(big main camera + a short isometric attitude strip) on the left, and the
situational-awareness window (corner camera over the top-down map) on the right.
RViz's 3D view is always the Qt central widget and Qt gives the window corners to
the top/bottom dock areas, so a single window cannot put the map in a corner --
hence two processes.

Both windows read fixed topics (/teleop/main_cam, /teleop/pip_cam). Which physical
camera feeds which pane is decided by two topic_tools/mux nodes, so swapping the
front and back cameras is a service call rather than a config edit. See
scripts/cam_swap.sh.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Screen layout, in pixels. Window A is flush left, window B picks up where it
# ends. Adjust both blocks together if you move to a different resolution.
MAIN_GEOMETRY = (0, 0, 1370, 1040)
MAP_GEOMETRY = (1370, 0, 550, 1040)

FRONT_BASE = "/depth_camera_front/color/image_raw"
REAR_BASE = "/depth_camera_rear/color/image_raw"


def launch_setup(context, *args, **kwargs):
    basestation_pkg_path = get_package_share_directory("basestation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    # Resolved to a plain string so the mux topic names can be built with f-strings.
    transport = LaunchConfiguration("transport").perform(context)

    front = f"{FRONT_BASE}/{transport}"
    rear = f"{REAR_BASE}/{transport}"
    main_out = f"/teleop/main_cam/image_raw/{transport}"
    pip_out = f"/teleop/pip_cam/image_raw/{transport}"

    # Gazebo only relays the compressed transport, so the sim needs its own pair of
    # configs with the two image topic suffixes changed.
    suffix = "" if transport == "ffmpeg" else "_sim"

    def rviz(name, config):
        return Node(
            package="rviz2",
            executable="rviz2",
            name=name,
            arguments=["-d", os.path.join(basestation_pkg_path, "rviz", config)],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        )

    def mux(name, output_topic, initial_topic):
        return Node(
            package="topic_tools",
            executable="mux",
            name=name,
            parameters=[{
                "input_topics": [front, rear],
                "output_topic": output_topic,
                "initial_topic": initial_topic,
                # Eager, so a swap does not have to set up a subscription and wait
                # for the next keyframe on top of that.
                "lazy": False,
                "use_sim_time": use_sim_time,
            }],
            output="screen",
        )

    return [
        rviz("rviz2_teleop_main", f"teleop_main{suffix}.rviz"),
        rviz("rviz2_teleop_map", f"teleop_map{suffix}.rviz"),
        mux("teleop_main_cam_mux", main_out, front),
        mux("teleop_pip_cam_mux", pip_out, rear),
        # Arena zones and mission waypoints, the one map-frame overlay that is
        # available without Nav2 running.
        Node(
            package="basestation",
            executable="zone_overlay",
            name="zone_overlay",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true.",
        ),
        DeclareLaunchArgument(
            "transport",
            default_value="ffmpeg",
            description="Camera transport to display: 'ffmpeg' on the rover, "
                        "'compressed' in sim (Gazebo does not publish ffmpeg).",
        ),
        OpaqueFunction(function=launch_setup),
    ])
