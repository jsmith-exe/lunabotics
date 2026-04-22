import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('qpl_rover')

    use_sim_time = LaunchConfiguration('use_sim_time')
    perception_params_file = LaunchConfiguration('perception_params_file')
    odom_params_file = LaunchConfiguration('odom_params_file')

    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    perception_params_arg = DeclareLaunchArgument(
        'perception_params_file',
        default_value=os.path.join(
            bringup_dir, 'config', 'vslam_perception_params.yaml'
        ),
        description='Full path to RTAB-Map perception params file'
    )

    odom_params_arg = DeclareLaunchArgument(
        'odom_params_file',
        default_value=os.path.join(
            bringup_dir, 'config', 'vslam_odom_params.yaml'
        ),
        description='Full path to RTAB-Map RGBD odometry params file'
    )

    # -------------------------------------------------------------------------
    # Front RGB-D sync
    # -------------------------------------------------------------------------
    rgbd_sync_front = Node(
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
    )

    # -------------------------------------------------------------------------
    # Rear RGB-D sync
    # -------------------------------------------------------------------------
    rgbd_sync_rear = Node(
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
    )

    # -------------------------------------------------------------------------
    # Multi-RGBD sync
    # -------------------------------------------------------------------------
    rgbdx_sync = Node(
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
    )

    # -------------------------------------------------------------------------
    # RGB-D visual odometry
    # Produces visual odom for EKF fusion
    # -------------------------------------------------------------------------
    vodometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        output='screen',
        parameters=[
            odom_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('rgb/image',       '/depth_camera_front/image_raw'),
            ('depth/image',     '/depth_camera_front/depth/image_raw'),
            ('rgb/camera_info', '/depth_camera_front/camera_info'),
        ],
    )

    # -------------------------------------------------------------------------
    # RTAB-Map perception / mapping node
    # Consumes filtered odometry and RGBDImages, outputs perception products
    # -------------------------------------------------------------------------
    perception = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtab_perception',
        namespace='rtabmap',
        output='screen',
        parameters=[
            perception_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('rgbd_images', '/rgbd_images'),
            ('odom',        '/odometry/filtered'),
        ],
        arguments=['--delete_db_on_start'],
    )

    return LaunchDescription([
        use_sim_time_arg,
        odom_params_arg,
        perception_params_arg,
        rgbd_sync_front,
        rgbd_sync_rear,
        rgbdx_sync,
        vodometry,
        perception,
    ])