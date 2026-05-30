from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from os import path

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


rover_pkg: str = get_package_share_directory("qpl_rover")


def setup_components(context):
    """Separate function to evaluate whether to launch components."""
    run_components = LaunchConfiguration('run_components').perform(context)

    if run_components != "true":
        return []

    components = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(rover_pkg, "launch", "components.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false"
        }.items()
    )

    return [components]


def generate_launch_description():

    run_components_parameter = DeclareLaunchArgument(
        'run_components',
        default_value='true',
        description='Whether to run the rover with components.'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(rover_pkg, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "use_ros2_control": "true"
        }.items()
    )

    # Cameras
    use_low_quality = "true"
    realsense_launch_source = PythonLaunchDescriptionSource(path.join(rover_pkg, "launch", "camera_realsense.launch.py"))
    orbbec_launch_path_source = PythonLaunchDescriptionSource(path.join(rover_pkg, "launch", "camera_orbbec.launch.py"))
    realsense_launch = IncludeLaunchDescription(realsense_launch_source, launch_arguments={"use_low_quality": use_low_quality}.items())
    orbbec_launch = IncludeLaunchDescription(orbbec_launch_path_source, launch_arguments={"use_low_quality": use_low_quality}.items())

    rear_camera_tf_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0', '0','0','0',
                'camera_link_rear',
                'depth_camera_rear_link'],
    )

    front_camera_tf_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0', '0','0','0',
                'camera_link_front',
                'camera_camera_link_front'],
    )

    return LaunchDescription([
        run_components_parameter,
        rsp,
        OpaqueFunction(function=setup_components),
        realsense_launch,
        orbbec_launch,
        rear_camera_tf_transform,
        front_camera_tf_transform,
    ])