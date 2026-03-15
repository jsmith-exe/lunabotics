from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = get_package_share_directory('qpl_rover')

    params_file = os.path.join(
        pkg_path,
        'config',
        'pointcloud_to_scan.yaml'
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[params_file],
        remappings=[
            ('cloud_in', '/points'),
            ('scan', '/scan')
        ]
    )

    return LaunchDescription([
        pointcloud_to_scan
    ])