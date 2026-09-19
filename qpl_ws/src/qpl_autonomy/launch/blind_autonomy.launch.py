from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),

        Node(
            package="qpl_autonomy",
            executable="competition_autonomy_node",
            name="competition_autonomy",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])