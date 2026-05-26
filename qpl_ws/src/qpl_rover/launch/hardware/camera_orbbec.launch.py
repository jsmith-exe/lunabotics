from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    width = "1920"
    height = "1080"
    camera_params = {
        # 'color_width': "1920",
        # 'color_height': "1080",
        'depth_width': "1280",
        'depth_height': "1024",
        'depth_fps': '7',
        
        # 'color_format': 'MJPG',
        # 'color_qos': 'SENSOR_DATA',
        # 'depth_registration': 'true',
        # 'enable_colored_point_cloud': 'true',
        'camera_name': 'orrbec'
    }.items()

    orbbec_pkg_path = get_package_share_directory('orbbec_camera')
    orbbec_launch_path = os.path.join(orbbec_pkg_path, 'launch', 'astra_pro_plus.launch.py')
    return LaunchDescription([
        # These are set before the node starts, scoped to all nodes in this launch
        SetParameter(name='color.image_raw.ffmpeg.encoder',  value='libx264'),
        SetParameter(name='color.image_raw.ffmpeg.bit_rate', value=1000000),
        SetParameter(name='color.image_raw.ffmpeg.qmax',     value=40),
        SetParameter(name='color.image_raw.ffmpeg.gop_size', value=5),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch_path),
            launch_arguments=camera_params
        ),
    ])