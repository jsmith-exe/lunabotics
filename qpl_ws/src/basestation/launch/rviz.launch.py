from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


config_name = "default"


def generate_launch_description():
    basestation_pkg_path = get_package_share_directory("basestation")

    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true."
    )

    rviz_config = os.path.join(
        basestation_pkg_path,
        "rviz",
        f"{config_name}.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
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