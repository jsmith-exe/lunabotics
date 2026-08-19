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

    # NOTE: enabling this on hardware also requires the RealSense to be running
    # (realsense_launch is commented out of the returned LaunchDescription
    # below), since VO consumes its colour + aligned depth streams.
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
    # Full resolution: 1280x800x30 colour, 848x480x30 depth. aligned_depth_to_color
    # (needed by visual odometry) comes out at the COLOUR resolution, so this is
    # also what sets the VO workload. If the Jetson cannot hold frame rate, flip
    # this to "true" for 424x240x15 - no other change is required.
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
        # Splices the RealSense's own frame subtree (including the IMU and colour
        # optical frames) onto the URDF, so the EKF can transform
        # /camera/camera/imu into base_footprint and rgbd_odometry can resolve
        # base_footprint -> the colour image's frame.
        #
        # The child frame below is a best guess at the driver's naming and has
        # NOT been checked against hardware. If VO or the IMU sit silent, read
        # the real frame off the image header
        # (ros2 topic echo --once /depth_camera_front/color/image_raw --field header.frame_id)
        # and fix it HERE rather than in the URDF.
        front_camera_tf_transform,
    ])