"""Synthetic colour cameras, for network testing with no hardware attached.

Replaces camera_realsense.launch.py / camera_orbbec.launch.py: same topic names,
same resolutions, same encoder settings, so measurements carry over to the real
cameras. Only colour is generated - depth and point clouds are not, so this
understates total egress.

  ros2 launch qpl_rover camera_sim.launch.py
  ros2 launch qpl_rover camera_sim.launch.py camera:=rear pattern:=texture noise_fraction:=0.3
  ros2 launch qpl_rover camera_sim.launch.py use_low_quality:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Matches the path convention in camera_orbbec.launch.py; the calibration folder
# is not installed to the package share.
CALIBRATION_FOLDER = '/home/qpl/lunabotics/qpl_ws/src/qpl_rover/calibration'

# Resolutions mirror what the real drivers are configured for, so the generated
# load matches the cameras being stood in for.
CAMERAS = {
    'front': {
        'camera_name': 'depth_camera_front',
        'frame_id': 'camera_link_front',
        'high': (1280, 800, 30.0),   # RealSense colour tops out here
        'low': (424, 240, 15.0),
        'camera_info_url': '',       # no front calibration on disk; node synthesises one
        # ffmpeg settings from camera_realsense.launch.py
        'ffmpeg_high': {'encoder': 'libx264', 'bit_rate': 1000000, 'qmax': 40, 'gop_size': 10},
        'ffmpeg_low': {'encoder': 'libx264rgb', 'bit_rate': 1000000, 'qmax': 40, 'gop_size': 1,
                       'encoder_av_options': 'tune:zerolatency,preset:ultrafast'},
    },
    'rear': {
        'camera_name': 'depth_camera_rear',
        'frame_id': 'camera_link_rear',
        'high': (1920, 1080, 30.0),
        'low': (640, 480, 30.0),
        'camera_info_url': f'file://{CALIBRATION_FOLDER}/rear_calib_1920_cam_info.yaml',
        # ffmpeg settings from camera_orbbec.launch.py
        'ffmpeg_high': {'encoder': 'libx264', 'bit_rate': 1000000, 'qmax': 40, 'gop_size': 1,
                        'encoder_av_options': 'tune:zerolatency,preset:ultrafast'},
        'ffmpeg_low': {'encoder': 'libx264rgb', 'bit_rate': 1000000, 'qmax': 40, 'gop_size': 30,
                       'encoder_av_options': 'tune:zerolatency,preset:ultrafast'},
    },
}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='both',
                              description='front, rear or both'),
        DeclareLaunchArgument('use_low_quality', default_value='false',
                              description='Use the drivers\' low quality profiles.'),
        DeclareLaunchArgument('pattern', default_value='noise',
                              description='noise (worst case), texture (realistic) or gradient (floor)'),
        DeclareLaunchArgument('noise_fraction', default_value='1.0',
                              description='0..1 blend of noise over the pattern; sweeps best to worst case.'),
        DeclareLaunchArgument('scene_cut_period', default_value='0.0',
                              description='Seconds between forced scene cuts; 0 disables.'),
        DeclareLaunchArgument('enable_ffmpeg', default_value='true',
                              description='Run image_transport republish to produce /ffmpeg.'),
        DeclareLaunchArgument('enable_compressed', default_value='true',
                              description='Publish /compressed directly from the node.'),
        OpaqueFunction(function=build_cameras),
    ])


def build_cameras(context):
    which = LaunchConfiguration('camera').perform(context).lower()
    low = LaunchConfiguration('use_low_quality').perform(context).lower() == 'true'
    pattern = LaunchConfiguration('pattern').perform(context)
    noise_fraction = float(LaunchConfiguration('noise_fraction').perform(context))
    scene_cut_period = float(LaunchConfiguration('scene_cut_period').perform(context))
    enable_ffmpeg = LaunchConfiguration('enable_ffmpeg').perform(context).lower() == 'true'
    enable_compressed = LaunchConfiguration('enable_compressed').perform(context).lower() == 'true'

    selected = ['front', 'rear'] if which == 'both' else [which]
    actions = []

    for key in selected:
        if key not in CAMERAS:
            raise RuntimeError(f"camera must be front, rear or both (got '{key}')")
        cam = CAMERAS[key]
        width, height, fps = cam['low' if low else 'high']
        base = f"/{cam['camera_name']}/color/image_raw"

        actions.append(Node(
            package='qpl_rover',
            executable='camera_sim',
            name=f'camera_sim_{key}',
            output='screen',
            parameters=[{
                'camera_name': cam['camera_name'],
                'frame_id': cam['frame_id'],
                'width': width,
                'height': height,
                'fps': fps,
                'pattern': pattern,
                'noise_fraction': noise_fraction,
                'scene_cut_period': scene_cut_period,
                'publish_compressed': enable_compressed,
                'camera_info_url': cam['camera_info_url'],
                # Latency against these stamps is only meaningful on wall time.
                'use_sim_time': False,
            }],
        ))

        if enable_ffmpeg:
            # The same ffmpeg_image_transport plugin the drivers load, just hosted
            # here instead - its parameters sit under out.ffmpeg.* on this node.
            ffmpeg = cam['ffmpeg_low' if low else 'ffmpeg_high']
            actions.append(Node(
                package='image_transport',
                executable='republish',
                name=f'republish_ffmpeg_{key}',
                arguments=['raw', 'ffmpeg'],
                output='screen',
                remappings=[('in', base), ('out/ffmpeg', f'{base}/ffmpeg')],
                parameters=[{f'out.ffmpeg.{k}': v for k, v in ffmpeg.items()}],
            ))

    return actions
