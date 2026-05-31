from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time_parameter = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Enable if using simulation'
    )

    return LaunchDescription([
        use_sim_time_parameter,
        OpaqueFunction(function=opaque_generate_launch_description),
    ])

def opaque_generate_launch_description(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    print(f'use_sim_time: {use_sim_time}')

    package_name = 'qpl_rover'
    rover_pkg = get_package_share_directory(package_name)
    ekf_global_params = os.path.join(rover_pkg, 'config', 'ekf_global_params.yaml')

    # 1. Static Anchor: Where the tag exists in the world
    tag_to_map_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_tag',
        arguments=['0.055', '0.2', '0.3', '0', '0', '0', 'map', 'tag_0']
    )

    # 2. The Observer: Your new Pupil-AprilTags Python node
    apriltag_observer = Node(
        package=package_name,
        executable='apriltag_observer',
        name='apriltag_observer',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Global EKF: Calculates Map -> Odom
    ekf_global_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global',
                parameters=[ekf_global_params, {'use_sim_time': use_sim_time}],
                remappings=[('odometry/filtered', '/odometry/global')],
            )
        ],
    )

    return [
        tag_to_map_static,
        apriltag_observer,
        ekf_global_node,
    ]