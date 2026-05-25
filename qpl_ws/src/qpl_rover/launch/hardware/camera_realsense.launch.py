from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_params = {
        "camera.color.image_raw.ffmpeg.encoder": "libx264",
        "camera.color.image_raw.ffmpeg.bit_rate": 1000000,
        "camera.color.image_raw.ffmpeg.qmax": 51,
        "pointcloud__neon_.enable": True,
        "initial_reset": True,
        'enable_gyro': True,
        'enable_accel': True,
        'unite_imu_method': 2,
        # 'color_profile': '1280x720x30'
    }
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[camera_params],
        output='screen',
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'gain': 0.1,
            'zeta': 0.0,
            'fixed_frame': 'camera_frame',
            'remove_gravity_vector': False,
        }],
        remappings=[
            ('/imu/data_raw', '/camera/camera/imu'),
        ]
    )

    return LaunchDescription([
        camera_node,
        imu_filter,
    ])