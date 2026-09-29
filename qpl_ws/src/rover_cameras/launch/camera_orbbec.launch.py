"""Rear camera (Orbbec Astra Pro Plus): driver + teleop stream encoder in one container."""
import importlib.util
import os
import sys
from typing import Dict, List

from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

sys.path.append(os.path.dirname(os.path.abspath(__file__)))  # launch_utils.py sits beside this file
from launch_utils import INTRA_PROCESS, respawning_container, stream_encoder  # noqa: E402

direction = 'rear'  # front or rear
camera_name = f'depth_camera_{direction}'


def generate_launch_description() -> LaunchDescription:
    use_low_quality_parameter = DeclareLaunchArgument(
        'use_low_quality',
        default_value='false',
        description='Whether to run camera with low quality.'
    )

    rear_camera_tf_transform = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0',
                   'camera_link_rear', 'depth_camera_rear_link'],
    )

    return LaunchDescription([
        use_low_quality_parameter,
        OpaqueFunction(function=get_camera_launch),
        rear_camera_tf_transform,
    ])


def get_camera_launch(context: LaunchContext) -> List[Action]:
    """ Returns camera configuration depending on launch parameter """
    use_low_quality = LaunchConfiguration('use_low_quality').perform(context).lower() == 'true'
    params = get_camera_params(use_low_quality)
    logs: List[Action] = []

    # Same node and container names as astra_pro_plus.launch.py, but our own container so
    # the stream encoder can share the process and receive frames intra-process.
    camera = ComposableNode(
        package='orbbec_camera',
        plugin='orbbec_camera::OBCameraNodeDriver',
        name=camera_name,
        namespace=camera_name,
        parameters=[orbbec_parameters(context, params, logs)],
        extra_arguments=INTRA_PROCESS,
    )
    encoder = stream_encoder(camera_name, use_low_quality)

    logs.insert(0, LogInfo(msg=(
        f'Rear Orbbec ({"low" if use_low_quality else "high"} quality): colour '
        f'{params["color_width"]}x{params["color_height"]} {params["color_format"]}, depth '
        f'{params["depth_width"]}x{params["depth_height"]}@{params["depth_fps"]}, topics under '
        f'/{camera_name}; teleop stream on /{camera_name}/color/teleop_stream/ffmpeg')))
    return [*logs, *respawning_container('camera_container', camera_name, [camera, encoder])]


def orbbec_parameters(
    context: LaunchContext, overrides: Dict[str, str], logs: List[Action],
) -> Dict[str, TextSubstitution]:
    """Node parameters exactly as astra_pro_plus.launch.py would pass them.

    That launch file turns each of its declared arguments into a node parameter, so defaults
    come from its declarations and `overrides` replace them. Keys it does not declare were
    never passed to the node before, so they are dropped here too. Values are wrapped as
    substitutions so launch_ros infers their types (int, bool, ...) exactly as it did for
    the launch arguments; plain strings would reach the node as strings.
    """
    path = os.path.join(
        get_package_share_directory('orbbec_camera'), 'launch', 'astra_pro_plus.launch.py')
    spec = importlib.util.spec_from_file_location('astra_pro_plus_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    declared = [e for e in module.generate_launch_description().entities
                if isinstance(e, DeclareLaunchArgument)]

    params: Dict[str, TextSubstitution] = {}
    for arg in declared:
        if arg.name in overrides:
            value = overrides[arg.name]
        else:
            value = ''.join(s.perform(context) for s in arg.default_value)
        params[arg.name] = TextSubstitution(text=value)
    ignored = sorted(set(overrides) - set(params))
    if ignored:
        logs.append(LogInfo(msg=(
            f'Rear Orbbec: ignoring {", ".join(ignored)} - astra_pro_plus.launch.py does not '
            'declare them, so they never reached the driver')))
    return params


def get_camera_params(use_low_quality: bool) -> Dict[str, str]:
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

    # Color (visual) runs at full 1080p for AprilTag detection; depth runs at 480p@30
    # (its point cloud only feeds the Nav2 voxel layer at <=1.5 m range). If the camera
    # refuses 640x480@30 in Y11, fall back to Y12.
    color_width = '1920'
    color_height = '1080'
    color_format = 'RGB888'
    depth_width = '640'
    depth_height = '480'
    depth_fps = '30'
    depth_format = 'Y11'

    if use_low_quality:
        color_width = '640'
        color_height = '480'
        color_format = 'MJPG'
        depth_width = '320'
        depth_height = '240'
        depth_fps = '30'
        depth_format = 'Y11'

    calibration_folder = os.path.join(get_package_share_directory('rover_cameras'), 'calibration')
    camera_params = {
        'camera_name': camera_name,

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

        # 'enable_point_cloud': 'false',

        # Not declared by astra_pro_plus.launch.py, so these have never reached the driver.
        'enable_decimation_filter': 'true',
        'decimation_filter_scale': '50',

        'color_info_url': f'file://{calibration_folder}/rear_calib_{color_width}_cam_info.yaml',

        # Ignored: with intra-process comms the driver always publishes images reliable, depth
        # 10 (ob_camera_node.cpp replaces the image QoS with rmw_qos_profile_default).
        # 'color_qos': 'SENSOR_DATA',

        # 'depth_registration': 'true',
        # 'enable_colored_point_cloud': 'true',
    }
    return camera_params
