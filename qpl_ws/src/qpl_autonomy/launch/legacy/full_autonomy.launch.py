from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    config_path = os.path.join(
        get_package_share_directory('qpl_autonomy'),
        'config',
        'waypoints.yaml'
    )

    full_autonomy_node = Node(
        package='qpl_autonomy',
        executable='full_autonomy_node',
        name='full_autonomy_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_path': config_path
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        full_autonomy_node
    ])
