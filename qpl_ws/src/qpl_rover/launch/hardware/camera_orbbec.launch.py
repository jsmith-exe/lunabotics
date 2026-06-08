from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_low_quality_parameter = DeclareLaunchArgument(
        'use_low_quality',
        default_value='false',
        description='Whether to run camera with low quality.'
    )

    return LaunchDescription([
        use_low_quality_parameter,
        OpaqueFunction(function=get_camera_launch),
    ])


def get_camera_launch(context):
    """ Returns camera configuration depending on launch parameter """
    use_low_quality = LaunchConfiguration('use_low_quality').perform(context).lower() == 'true'
    print(f'use_low_quality: {use_low_quality}')

    if use_low_quality:
        ffmpeg_params = [
            SetParameter(name='color.image_raw.ffmpeg.encoder',  value='libx264rgb'),
            SetParameter(name='color.image_raw.ffmpeg.bit_rate', value=1000000),
            SetParameter(name='color.image_raw.ffmpeg.qmax',     value=40),
            SetParameter(name='color.image_raw.ffmpeg.gop_size', value=30), # Increases latency but lower bandwidth to counteract libx264rgb
            SetParameter(name='color.image_raw.ffmpeg.encoder_av_options', value='tune:zerolatency,preset:ultrafast'),
        ]
    else:
        ffmpeg_params = [
            SetParameter(name='color.image_raw.ffmpeg.encoder',  value='libx264'),
            SetParameter(name='color.image_raw.ffmpeg.bit_rate', value=1000000),
            SetParameter(name='color.image_raw.ffmpeg.qmax',     value=40),
            SetParameter(name='color.image_raw.ffmpeg.gop_size', value=1), # Minimise latency; does increase bandwidth
            SetParameter(name='color.image_raw.ffmpeg.encoder_av_options', value='tune:zerolatency,preset:ultrafast'),
        ]

    orbbec_pkg_path = get_package_share_directory('orbbec_camera')
    orbbec_launch_path = os.path.join(orbbec_pkg_path, 'launch', 'astra_pro_plus.launch.py')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch_path),
        launch_arguments=get_camera_params(use_low_quality).items()
    )

    return [*ffmpeg_params, camera_launch]


def get_camera_params(use_low_quality: bool):
    """
    Possible depth profiles:
     - 1280x1024 7fps
     - 640x480 30fps
     - 320x240 30fps
     - 160x120 30fps
    For V11 and V12 formats

    Possible color profiles:
     - 1920x1080 30fps
     - 1280x720 30fps
     - 640x480 30fps
     For MJPG, RGB888, BGRA
    """

    direction = 'rear' # front or rear

    color_width = '1920'
    color_height = '1080'
    color_format = 'RGB888'
    depth_width = '1280'
    depth_height = '1024'
    depth_fps = '7'
    depth_format = 'Y12'

    if use_low_quality:
        color_width = '640'
        color_height = '480'
        color_format = 'MJPG'
        depth_width = '320'
        depth_height = '240'
        depth_fps = '30'
        depth_format = 'Y11'

    calibration_folder = '/home/qpl/lunabotics/qpl_ws/src/qpl_rover/calibration'
    camera_params = {
        'camera_name': f'depth_camera_{direction}',

        'color_width': color_width,
        'color_height': color_height,
        'color_format': color_format,

        'depth_width': depth_width,
        'depth_height': depth_height,
        'depth_fps': depth_fps,
        'depth_format': depth_format,

        'ir_width': depth_width,
        'ir_height': depth_height,
        'ir_fps': depth_fps,
        'ir_format': 'Y10',

        'color_info_url': f'file://{calibration_folder}/rear_calib_{color_width}_cam_info.yaml',

        # 'color_qos': 'SENSOR_DATA',
        # 'depth_registration': 'true',
        # 'enable_colored_point_cloud': 'true',
    }

    print(camera_params)
    return camera_params
