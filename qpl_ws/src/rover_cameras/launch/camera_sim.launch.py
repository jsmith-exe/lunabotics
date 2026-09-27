"""Synthetic colour cameras, for network testing with no hardware attached.

Replaces camera_realsense.launch.py / camera_orbbec.launch.py: same topic names,
same resolutions, same stream encoder settings, so measurements carry over to the
real cameras. Only colour is generated - depth and point clouds are not, so this
understates total egress. The encoder runs as a separate process here (the sim is
Python, so it cannot share a container), which costs a local copy per frame.

  ros2 launch rover_cameras camera_sim.launch.py
  ros2 launch rover_cameras camera_sim.launch.py camera:=rear pattern:=texture noise_fraction:=0.3
  ros2 launch rover_cameras camera_sim.launch.py use_low_quality:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_cameras.launch_utils import stream_encoder_process

CALIBRATION_FOLDER = os.path.join(get_package_share_directory('rover_cameras'), 'calibration')

# Resolutions mirror what the real drivers are configured for, so the generated
# load matches the cameras being stood in for.
CAMERAS = {
    'front': {
        'camera_name': 'depth_camera_front',
        'frame_id': 'camera_link_front',
        'high': (1280, 800, 30.0),   # RealSense colour tops out here
        'low': (424, 240, 15.0),
        'camera_info_url': '',       # no front calibration on disk; node synthesises one
    },
    'rear': {
        'camera_name': 'depth_camera_rear',
        'frame_id': 'camera_link_rear',
        'high': (1920, 1080, 30.0),
        'low': (640, 480, 30.0),
        'camera_info_url': f'file://{CALIBRATION_FOLDER}/rear_calib_1920_cam_info.yaml',
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
        DeclareLaunchArgument('enable_stream', default_value='true',
                              description='Run the stream encoder to produce color/teleop_stream/ffmpeg.'),
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
    enable_stream = LaunchConfiguration('enable_stream').perform(context).lower() == 'true'
    enable_compressed = LaunchConfiguration('enable_compressed').perform(context).lower() == 'true'

    selected = ['front', 'rear'] if which == 'both' else [which]
    actions = []

    for key in selected:
        if key not in CAMERAS:
            raise RuntimeError(f"camera must be front, rear or both (got '{key}')")
        cam = CAMERAS[key]
        width, height, fps = cam['low' if low else 'high']

        actions.append(Node(
            package='rover_cameras',
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

        if enable_stream:
            # The same encoder and settings the camera launch files load beside the drivers.
            actions.append(stream_encoder_process(cam['camera_name'], low))

    return actions
