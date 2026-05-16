from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # These are set before the node starts, scoped to all nodes in this launch
        SetParameter(name='color.image_raw.ffmpeg.encoder',  value='libx264'),
        SetParameter(name='color.image_raw.ffmpeg.bit_rate', value=1000000),
        SetParameter(name='color.image_raw.ffmpeg.qmax',     value=40),
        SetParameter(name='color.image_raw.ffmpeg.gop_size', value=5),
        
        # SetParameter(name='color.image_raw.ffmpeg.encoder',            value='libx264'),
        # SetParameter(name='color.image_raw.ffmpeg.bit_rate',           value=4000000),
        # SetParameter(name='color.image_raw.ffmpeg.gop_size',           value=1),
        # SetParameter(name='color.image_raw.ffmpeg.max_b_frames',       value=0),        # B-frames require buffering future frames
        # SetParameter(name='color.image_raw.ffmpeg.encoder_av_options', value='preset=ultrafast:tune=zerolatency'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                )
            ),
        ),
    ])