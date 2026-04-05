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

        # ---------------------------------------------------------------------
        # Front RGB-D sync
        # ---------------------------------------------------------------------
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync_front',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('rgb/image',       '/depth_camera_front/image_raw'),
                ('rgb/camera_info', '/depth_camera_front/camera_info'),
                ('depth/image',     '/depth_camera_front/depth/image_raw'),
                ('rgbd_image',      '/front/rgbd_image'),
            ],
        ),

        # ---------------------------------------------------------------------
        # Rear RGB-D sync
        # ---------------------------------------------------------------------
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync_rear',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('rgb/image',       '/depth_camera_rear/image_raw'),
                ('rgb/camera_info', '/depth_camera_rear/camera_info'),
                ('depth/image',     '/depth_camera_rear/depth/image_raw'),
                ('rgbd_image',      '/rear/rgbd_image'),
            ],
        ),

        # ---------------------------------------------------------------------
        # Multi-RGBD sync
        # Synchronises front + rear RGBDImage into one RGBDImages topic
        # ---------------------------------------------------------------------
        Node(
            package='rtabmap_sync',
            executable='rgbdx_sync',
            name='rgbdx_sync',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('rgbd_image0', '/front/rgbd_image'),
                ('rgbd_image1', '/rear/rgbd_image'),
                ('rgbd_images', '/rgbd_images'),
            ],
        ),

        # ---------------------------------------------------------------------
        # RTAB-Map SLAM
        # Uses RGBDImages interface because this build does not support
        # direct multi-rgbd sync inside rtabmap.
        # ---------------------------------------------------------------------
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
                ('rgbd_images', '/rgbd_images'),
                ('odom',        '/odometry/filtered'),
            ],
            arguments=['--delete_db_on_start'],
        ),
    ])