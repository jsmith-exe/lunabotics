"""Launch helpers shared by the camera launch files (and qpl_rover's sim.launch.py).

Not a launch file itself. Launch files import it from the directory they are installed in:

    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from launch_utils import ...
"""
import os
from typing import Any, Dict, List, Optional, Sequence, Union

from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchContext
from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.events.process import ProcessStarted
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

# One entry of a node's `parameters`: a YAML file path or a dict of values.
ParameterEntry = Union[str, Dict[str, Any]]

# With intra-process comms, both drivers (realsense2_camera, orbbec_camera) publish images
# through a plain rclcpp publisher instead of image_transport, with the default QoS
# (reliable, depth 10). So their colour image is raw only - no /compressed - and their
# image QoS settings (color_qos) and transport-plugin parameters are ignored. The
# basestation views <camera>/color/teleop_stream/ffmpeg from the stream encoder instead.
INTRA_PROCESS: List[Dict[str, bool]] = [{'use_intra_process_comms': True}]

# Low quality mode already shrinks the colour stream at the driver (424x240x15 front,
# 640x480 rear), so the encoder keeps that size and spends fewer bits.
LOW_QUALITY_ENCODER: Dict[str, Any] = {
    'width': 0, 'height': 0, 'bit_rate': 400000, 'max_fps': 15.0}


def config_path(name: str) -> str:
    return os.path.join(get_package_share_directory('rover_cameras'), 'config', name)


def stream_encoder_parameters(camera: str, use_low_quality: bool) -> List[ParameterEntry]:
    """Parameters for <camera>'s stream encoder: config/<front|rear>_stream.yaml + mode."""
    config = 'front_stream.yaml' if camera == 'depth_camera_front' else 'rear_stream.yaml'
    return [config_path(config), LOW_QUALITY_ENCODER if use_low_quality else {}]


def stream_encoder(camera: str, use_low_quality: bool) -> ComposableNode:
    """Stream encoder component for <camera>, for loading beside its driver."""
    return ComposableNode(
        package='rover_cameras',
        plugin='rover_cameras::StreamEncoder',
        name='stream_encoder',
        namespace=camera,
        parameters=stream_encoder_parameters(camera, use_low_quality),
        extra_arguments=INTRA_PROCESS,
    )


def stream_encoder_process(
    camera: str,
    use_low_quality: bool = False,
    overrides: Optional[Dict[str, Any]] = None,
) -> Node:
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


def gazebo_stream_encoders(
    cameras: Sequence[str] = ('depth_camera_front', 'depth_camera_rear'),
) -> List[Node]:
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


def respawning_container(
    name: str,
    namespace: str,
    nodes: List[ComposableNode],
    reload_delay: float = 12.0,
) -> List[Action]:
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

    def on_start(event: ProcessStarted, context: LaunchContext) -> List[Action]:
        starts[0] += 1
        load = LoadComposableNodes(target_container=container, composable_node_descriptions=nodes)
        if starts[0] == 1:
            return [load]
        return [
            LogInfo(msg=f'/{namespace}/{name} restarted after exiting; reloading the camera '
                        f'driver and stream encoder in {reload_delay:g} s (waiting for the old '
                        'process to drop off DDS)'),
            TimerAction(period=reload_delay, actions=[load]),
        ]

    return [RegisterEventHandler(OnProcessStart(target_action=container, on_start=on_start)),
            container]
