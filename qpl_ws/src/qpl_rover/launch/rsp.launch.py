import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # <-- add this


def generate_launch_description():
    package = "qpl_rover"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    pkg_share = get_package_share_directory(package)
    xacro_file = os.path.join(pkg_share, "description", "rover.urdf.xacro")

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_ros2_control:=", use_ros2_control,
            " sim_mode:=", use_sim_time,
        ]),
        value_type=str
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use sim time if true",
        ),
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="true",
            description="Use ros2_control if true",
        ),
        node_robot_state_publisher,
    ])