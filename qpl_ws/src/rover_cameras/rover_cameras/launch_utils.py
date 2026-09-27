"""Launch helpers shared by the camera launch files."""
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

# Drivers keep raw (for on-board consumers) and compressed (for local debugging);
# the basestation views <camera>/color/teleop_stream/ffmpeg from the stream encoder instead.
DRIVER_IMAGE_PLUGINS = ['image_transport/raw', 'image_transport/compressed']

INTRA_PROCESS = [{'use_intra_process_comms': True}]


def config_path(name):
    return os.path.join(get_package_share_directory('rover_cameras'), 'config', name)


# Low quality mode already shrinks the colour stream at the driver (424x240x15 front,
# 640x480 rear), so the encoder keeps that size and spends fewer bits.
LOW_QUALITY_ENCODER = {'width': 0, 'height': 0, 'bit_rate': 400000, 'max_fps': 15.0}


def stream_encoder_parameters(camera, use_low_quality):
    """Parameters for <camera>'s stream encoder: config/<front|rear>_stream.yaml + mode."""
    config = 'front_stream.yaml' if camera == 'depth_camera_front' else 'rear_stream.yaml'
    return [config_path(config), LOW_QUALITY_ENCODER if use_low_quality else {}]


def stream_encoder(camera, use_low_quality):
    """Stream encoder component for <camera>, for loading beside its driver."""
    return ComposableNode(
        package='rover_cameras',
        plugin='rover_cameras::StreamEncoder',
        name='stream_encoder',
        namespace=camera,
        parameters=stream_encoder_parameters(camera, use_low_quality),
        extra_arguments=INTRA_PROCESS,
    )


def stream_encoder_process(camera, use_low_quality=False, overrides=None):
    """Stream encoder for <camera> as its own process.

    For image sources that can't share a container: camera_sim (Python) and Gazebo. Frames
    arrive over local DDS instead of intra-process, which costs a copy per frame.
    `overrides` replace YAML values, e.g. input topics or use_sim_time.
    """
    return Node(
        package='rover_cameras',
        executable='stream_encoder_node',
        name='stream_encoder',
        namespace=camera,
        output='screen',
        parameters=stream_encoder_parameters(camera, use_low_quality) + [overrides or {}],
    )


def gazebo_stream_encoders(cameras=('depth_camera_front', 'depth_camera_rear')):
    """Stream encoders for the Gazebo cameras, on the same output topics as the rover.

    gazebo_ros_camera publishes <camera>/image_raw and <camera>/camera_info (sim.launch.py
    relays them to <camera>/color/...); the encoder reads the originals to skip the relay.
    """
    return [
        stream_encoder_process(camera, overrides={
            'input_topic': f'/{camera}/image_raw',
            'camera_info_topic': f'/{camera}/camera_info',
            'use_sim_time': True,
        })
        for camera in cameras
    ]


def respawning_container(name, namespace, nodes, reload_delay=12.0):
    """A component container that loads `nodes` every time its process starts.

    ComposableNodeContainer only loads its components once, so after a respawn the
    container would come back empty. Loading on each ProcessStarted fixes that.

    After a crash the dead container's load_node service stays discoverable until its DDS
    lease expires (~10 s with Cyclone's defaults). A load sent before then goes to the dead
    process and never returns, so reloads after a respawn wait `reload_delay` seconds.
    """
    container = ComposableNodeContainer(
        name=name,
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )
    starts = [0]

    def on_start(event, context):
        starts[0] += 1
        load = LoadComposableNodes(target_container=container, composable_node_descriptions=nodes)
        if starts[0] == 1:
            return [load]
        return [TimerAction(period=reload_delay, actions=[load])]

    return [RegisterEventHandler(OnProcessStart(target_action=container, on_start=on_start)),
            container]
