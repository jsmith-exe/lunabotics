from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    pkg = get_package_share_directory("luna_rviz")

    #xacro_path = os.path.join(pkg, "description", "rover.urdf.xacro")
    rviz_path = os.path.join(pkg, "rviz", "rover.rviz")

    '''robot_description = ParameterValue(
        Command(["xacro", " ", xacro_path]),
        value_type=str
    )'''

    # RViz (loads your config)
    rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_path],
            output="screen",
        )

    return LaunchDescription([
        rviz,
        
        
    ])