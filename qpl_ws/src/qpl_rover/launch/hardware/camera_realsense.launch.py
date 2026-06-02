from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


direction = 'front' # front or rear

def generate_launch_description():
    use_low_quality_parameter = DeclareLaunchArgument(
        'use_low_quality',
        default_value='false',
        description='Whether to run camera with low quality.'
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
        use_low_quality_parameter,
        OpaqueFunction(function=get_camera_launch),
        imu_filter,
    ])


def get_camera_launch(context):
    """ Returns camera configuration depending on launch parameter """
    use_low_quality = LaunchConfiguration('use_low_quality').perform(context).lower() == 'true'
    print(f'use_low_quality: {use_low_quality}')

    camera_launch = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[get_camera_params(use_low_quality)],
            output='screen',
            remappings=get_remappings()
        )

    return [camera_launch]


def get_remappings():
    # Use ros2 topic list to get topics and paste them here, do not include points (/camera/camera/depth/color/points)
    topics_to_remap = """
    /camera/camera/accel/imu_info
    /camera/camera/accel/metadata
    /camera/camera/accel/sample
    /camera/camera/color/camera_info
    /camera/camera/color/image_raw
    /camera/camera/color/image_raw/compressed
    /camera/camera/color/image_raw/compressedDepth
    /camera/camera/color/image_raw/ffmpeg
    /camera/camera/color/image_raw/theora
    /camera/camera/color/metadata
    /camera/camera/depth/camera_info
    /camera/camera/depth/image_rect_raw
    /camera/camera/depth/image_rect_raw/compressed
    /camera/camera/depth/image_rect_raw/compressedDepth
    /camera/camera/depth/image_rect_raw/ffmpeg
    /camera/camera/depth/image_rect_raw/theora
    /camera/camera/depth/metadata
    /camera/camera/extrinsics/depth_to_accel
    /camera/camera/extrinsics/depth_to_color
    /camera/camera/extrinsics/depth_to_gyro
    /camera/camera/extrinsics/depth_to_infra1
    /camera/camera/extrinsics/depth_to_infra2
    /camera/camera/gyro/imu_info
    /camera/camera/gyro/metadata
    /camera/camera/gyro/sample
    /camera/camera/infra1/camera_info
    /camera/camera/infra1/image_rect_raw
    /camera/camera/infra1/image_rect_raw/compressed
    /camera/camera/infra1/image_rect_raw/compressedDepth
    /camera/camera/infra1/image_rect_raw/ffmpeg
    /camera/camera/infra1/image_rect_raw/theora
    /camera/camera/infra1/metadata
    /camera/camera/infra2/camera_info
    /camera/camera/infra2/image_rect_raw
    /camera/camera/infra2/image_rect_raw/compressed
    /camera/camera/infra2/image_rect_raw/compressedDepth
    /camera/camera/infra2/image_rect_raw/ffmpeg
    /camera/camera/infra2/image_rect_raw/theora
    /camera/camera/infra2/metadata
    """.strip().split()

    topic_name_base = f'/depth_camera_{direction}'
    # Generate remappings
    remappings=[(topic, topic.replace(f'/camera/camera', topic_name_base)) for topic in topics_to_remap]
    # Additional remappings
    remappings += [
        (f'/camera/camera/depth/color/points', f'{topic_name_base}/depth/points'),
    ]

    print('Remappings:')
    [print(remapping) for remapping in remappings]
    return remappings


def get_camera_params(use_low_quality: bool):
    """
    STREAM      RESOLUTION     FORMAT                                FPS
    Infrared    1280x720       UYVY, BGRA8, RGBA8, BGR8, RGB8        @ 30/15/5 Hz
    Infrared     848x480       UYVY, BGRA8, RGBA8, BGR8, RGB8        @ 90/60/30/15/5 Hz
    Infrared     848x100       UYVY, BGRA8, RGBA8, BGR8, RGB8        @ 100 Hz
    Infrared     640x480       UYVY, BGRA8, RGBA8, BGR8, RGB8        @ 90/60/30/15/5 Hz
    Infrared     640x360       UYVY, BGRA8, RGBA8, BGR8, RGB8        @ 90/60/30/15/5 Hz
    Infrared     480x270       UYVY, BGRA8, RGBA8, BGR8, RGB8        @ 90/60/30/15/5 Hz
    Infrared     424x240       UYVY, BGRA8, RGBA8, BGR8, RGB8        @ 90/60/30/15/5 Hz

    Depth       1280x720       Z16                                   @ 30/15/5 Hz
    Depth        848x480       Z16                                   @ 90/60/30/15/5 Hz
    Depth        848x100       Z16                                   @ 100 Hz
    Depth        640x480       Z16                                   @ 90/60/30/15/5 Hz
    Depth        640x360       Z16                                   @ 90/60/30/15/5 Hz
    Depth        480x270       Z16                                   @ 90/60/30/15/5 Hz
    Depth        424x240       Z16                                   @ 90/60/30/15/5 Hz
    Depth        256x144       Z16                                   @ 90 Hz

    Color       1280x800       RGB8, BGRA8, RGBA8, BGR8, YUYV        @ 30/15/10/5 Hz
    Color       1280x720       RGB8, BGRA8, RGBA8, BGR8, YUYV        @ 30/15/10/5 Hz
    Color        848x480       RGB8, BGRA8, RGBA8, BGR8, YUYV        @ 60/30/15/5 Hz
    Color        640x480       RGB8, BGRA8, RGBA8, BGR8, YUYV        @ 60/30/15/5 Hz
    Color        640x360       RGB8, BGRA8, RGBA8, BGR8, YUYV        @ 90/60/30/15/5 Hz
    Color        480x270       RGB8, BGRA8, RGBA8, BGR8, YUYV        @ 90/60/30/15/5 Hz
    Color        424x240       RGB8, BGRA8, RGBA8, BGR8, YUYV        @ 90/60/30/15/5 Hz
    """

    profile = '1280x720x30'
    color_fmt = 'BGR8'
    depth_fmt = 'Z16'
    infra_fmt = 'BGR8'

    if use_low_quality:
        profile = '424x240x15'
        color_fmt = 'BGR8'
        infra_fmt = 'UYVY'

    camera_params = {
        'rgb_camera.color_profile': profile,
        'depth_module.depth_profile': profile,
        'depth_module.infra_profile': profile,
        'rgb_camera.color_format': color_fmt,
        'depth_module.depth_format': depth_fmt,
        'depth_module.infra_format': infra_fmt,

        f'camera.color.image_raw.ffmpeg.encoder': 'libx264',
        f'camera.color.image_raw.ffmpeg.bit_rate': 1000000,
        f'camera.color.image_raw.ffmpeg.qmax': 51,

        'pointcloud__neon_.enable': True,
        # 'align_depth.enable': True,

        'enable_gyro': True,
        'enable_accel': True,
        'unite_imu_method': 2,

        'initial_reset': True,
        'base_frame_id': 'camera_link_front',
    }

    print(camera_params)
    return camera_params
