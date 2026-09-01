from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from os import path

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, TimerAction
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
            "use_sim_time": "false",
            "use_vslam": LaunchConfiguration("use_vslam").perform(context),
        }.items()
    )

    return [components]


def generate_launch_description():

    run_components_parameter = DeclareLaunchArgument(
        'run_components',
        default_value='true',
        description='Whether to run the rover with components.'
    )

    # VO consumes the RealSense's colour + aligned depth streams, so this
    # depends on realsense_launch below.
    use_vslam_parameter = DeclareLaunchArgument(
        'use_vslam',
        default_value='false',
        description='Run RGB-D visual odometry and fuse it into the local EKF as odom1.'
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
    # Full resolution: 1280x800x30 colour, 848x480x30 depth. This also sets the
    # VO workload. If the Jetson cannot hold frame rate, flip to "true" for
    # 424x240x15 - no other change is required.
    use_low_quality = "false"
    realsense_launch_source = PythonLaunchDescriptionSource(path.join(rover_pkg, "launch", "camera_realsense.launch.py"))
    orbbec_launch_path_source = PythonLaunchDescriptionSource(path.join(rover_pkg, "launch", "camera_orbbec.launch.py"))
    realsense_launch = IncludeLaunchDescription(realsense_launch_source, launch_arguments={"use_low_quality": use_low_quality}.items())
    orbbec_launch = IncludeLaunchDescription(orbbec_launch_path_source, launch_arguments={"use_low_quality": use_low_quality}.items())
    delayed_orbbec_launch = TimerAction(period=20.0, actions=[orbbec_launch])

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
        use_vslam_parameter,
        rsp,
        OpaqueFunction(function=setup_components),
        realsense_launch,
        # delayed_orbbec_launch,
        # rear_camera_tf_transform,
        # Splices the RealSense's frame subtree onto the URDF so the EKF can
        # transform /camera/camera/imu into base_footprint and rgbd_odometry can
        # resolve base_footprint -> the colour image's frame.
        #
        # The child frame is derived: realsense2_camera prefixes camera_name onto
        # base_frame_id, so "camera" + "camera_link_front" gives
        # "camera_camera_link_front". If base_frame_id in
        # camera_realsense.launch.py changes, or camera_name is ever set, this
        # must change with it or VO/IMU go silent with no error.
        front_camera_tf_transform,
    ])