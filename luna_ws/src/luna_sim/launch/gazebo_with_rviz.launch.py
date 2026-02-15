from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    gazebo_launchfile_path = os.path.join(get_package_share_directory('luna_sim'), 'launch', 'gazebo_rover.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launchfile_path)
    )

    rviz_launchfile_path = os.path.join(get_package_share_directory('luna_rviz'), 'launch', 'rviz.launch.py')
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launchfile_path)
    )

    return LaunchDescription([
        gazebo,
        rviz,
    ])