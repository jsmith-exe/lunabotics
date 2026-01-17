from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    pkg = get_package_share_directory("lunabotics_description")

    xacro_path = os.path.join(pkg, "urdf", "lunabotics.urdf.xacro")
    rviz_path = os.path.join(pkg, "rviz", "rover.rviz")

    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_path]),
        value_type=str
    )

    return LaunchDescription([
        # Publishes TF base_link -> wheels etc. from robot_description + /joint_states
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),

        # Diff-drive sim: subscribes /cmd_vel, publishes /odom, /joint_states, TF odom->base_link
        Node(
            package="lunabotics_control",
            executable="diffdrive_sim",
            name="diffdrive_sim",
            output="screen",
            # If you made cmd_vel relative ("cmd_vel"), you can remap like this:
            # remappings=[("cmd_vel", "/cmd_vel")],
        ),

        # RViz (loads your config)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_path],
            output="screen",
        ),
    ])
