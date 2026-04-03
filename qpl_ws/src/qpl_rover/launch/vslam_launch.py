import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('qpl_rover')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'vslam_params.yaml'),
            description='Full path to rtabmap params file'
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('rgb/image',       '/depth_camera_front/image_raw'),
                ('rgb/camera_info', '/depth_camera_front/camera_info'),
                ('depth/image',     '/depth_camera_front/depth/image_raw'),
                ('odom',            '/odometry/filtered'),
            ],
            arguments=['--delete_db_on_start'],
        ),

        # Node(
        #     package='rtabmap_viz',
        #     executable='rtabmap_viz',
        #     name='rtabmap_viz',
        #     namespace='rtabmap',
        #     output='screen',
        #     parameters=[
        #         {'use_sim_time': use_sim_time},
        #     ],
        #     remappings=[
        #         ('rgb/image',       '/depth_camera_front/image_raw'),
        #         ('rgb/camera_info', '/depth_camera_front/camera_info'),
        #         ('depth/image',     '/depth_camera_front/depth/image_raw'),
        #     ],
        # ),

    ])