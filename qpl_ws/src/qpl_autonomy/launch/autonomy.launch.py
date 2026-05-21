from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('qpl_autonomy'),
        'config',
        'waypoints.yaml'
    )

    autonomy_node = Node(
        package='qpl_autonomy',
        executable='autonomy_node',
        name='autonomy_node',
        output='screen',
        parameters=[{
            'config_path': config_path
        }]
    )

    return LaunchDescription([
        autonomy_node
    ])