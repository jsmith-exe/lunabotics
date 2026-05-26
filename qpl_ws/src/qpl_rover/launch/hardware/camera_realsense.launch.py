from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    direction = 'front' # front or rear

    profile = '1280x720x30'

    namespace = 'rover'
    name = 'realsense'
    camera_params = {
        'camera_namespace': namespace,
        'camera_name': name,
        'color_profile': profile,
        'depth_profile': profile,
        f'{name}.color.image_raw.ffmpeg.encoder': 'libx264',
        f'{name}.color.image_raw.ffmpeg.bit_rate': 1000000,
        f'{name}.color.image_raw.ffmpeg.qmax': 51,
        'pointcloud__neon_.enable': True,
        'initial_reset': True,
        'enable_gyro': True,
        'enable_accel': True,
    }

    # Batch remapping
    # Use ros2 topic list to get topics and paste them here, do not include points (/rover/realsense/depth/color/points)
    topics_to_remap = """
    /rover/realsense/color/camera_info
    /rover/realsense/color/image_raw
    /rover/realsense/color/image_raw/compressed
    /rover/realsense/color/image_raw/compressedDepth
    /rover/realsense/color/image_raw/ffmpeg
    /rover/realsense/color/image_raw/theora
    /rover/realsense/color/metadata
    /rover/realsense/depth/camera_info
    /rover/realsense/depth/image_rect_raw
    /rover/realsense/depth/image_rect_raw/compressed
    /rover/realsense/depth/image_rect_raw/compressedDepth
    /rover/realsense/depth/image_rect_raw/ffmpeg
    /rover/realsense/depth/image_rect_raw/theora
    /rover/realsense/depth/metadata
    /rover/realsense/extrinsics/depth_to_color
    /rover/realsense/extrinsics/depth_to_infra1
    /rover/realsense/extrinsics/depth_to_infra2
    /rover/realsense/infra1/camera_info
    /rover/realsense/infra1/image_rect_raw
    /rover/realsense/infra1/image_rect_raw/compressed
    /rover/realsense/infra1/image_rect_raw/compressedDepth
    /rover/realsense/infra1/image_rect_raw/ffmpeg
    /rover/realsense/infra1/image_rect_raw/theora
    /rover/realsense/infra1/metadata
    /rover/realsense/infra2/camera_info
    /rover/realsense/infra2/image_rect_raw
    /rover/realsense/infra2/image_rect_raw/compressed
    /rover/realsense/infra2/image_rect_raw/compressedDepth
    /rover/realsense/infra2/image_rect_raw/ffmpeg
    /rover/realsense/infra2/image_rect_raw/theora
    /rover/realsense/infra2/metadata
    """.strip().split()

    # Generate remappings
    topic_name_base = f'/depth_camera_{direction}'
    remappings=[(topic, topic.replace(f'/{namespace}/{name}', topic_name_base)) for topic in topics_to_remap] + [
        # Additional remappings
        (f'/rover/realsense/depth/color/points', f'{topic_name_base}/depth/points'),
    ]

    print('Remappings:')
    [print(remapping) for remapping in remappings]
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=name,
            namespace=namespace,
            parameters=[camera_params],
            output='screen',
            remappings=remappings
        )
    ])