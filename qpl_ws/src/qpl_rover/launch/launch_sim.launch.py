from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():

    package_name = "qpl_rover"

    world_path = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "april.world"
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), "config", "gazebo_params.yaml")

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={"world": world_path, 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "rover"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    ekf_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "imu_params.yaml",
    )

    ekf_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params, {"use_sim_time": True}],
            )
        ],
    )

    apriltag_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'apriltag.yaml'
    )

    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        remappings=[
            ('image', '/back_camera/image_raw'),
            ('camera_info', '/back_camera/camera_info'),
            ('image_rect', '/back_camera/image_rect'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    apriltag_node = Node(
    package='apriltag_ros',
    executable='apriltag_node',
    name='apriltag_node',
    remappings=[
        ('image_rect', '/back_camera/image_rect'),
        ('camera_info', '/back_camera/camera_info'),
    ],
    parameters=[apriltag_config, {
        'use_sim_time': True,
        'queue_size': 10,
        'sync_slop': 0.05,
    }],
)

    return LaunchDescription([
        rsp,
        ekf_node,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        rectify_node,
        apriltag_node,
    ])