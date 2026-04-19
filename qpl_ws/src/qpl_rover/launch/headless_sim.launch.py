from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():

    package_name = "qpl_rover"

    # tell gazebo where to find the apriltag model so the texture loads on any machine
    models_path = os.path.join(get_package_share_directory(package_name), "worlds")
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    world_path = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "arena_april.world"
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), "launch", "rsp.launch.py"
        )]),
        launch_arguments={
            "use_sim_time": "true",
            "use_ros2_control": "true"
        }.items()
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), "config", "gazebo_params.yaml"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"
        )]),
        launch_arguments={
            "world": world_path,
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file
        }.items()
    )

    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic", "robot_description",
                    "-entity", "rover",
                    "-x", "2.0", # set to 0 (default) if not using april_arena.world
                    "-y", "1.0", # set to 0 (default) if not using april_arena.world
                    "-z", "0.2"], # set to 0 (default) if not using april_arena.world
                output="screen",
            )
        ],
    )


    controller_spawners = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont"],
            ),
        ]
    )

    ekf_local_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "ekf_local_params.yaml",
    )

    # Delay EKF until after controllers are up (period=4.0) so the robot exists
    # and /diff_cont/odom + /imu/data are already publishing before EKF initialises.
    # Starting EKF too early causes a bad initial state that never recovers cleanly.
    ekf_local_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_local",
                output="screen",
                parameters=[ekf_local_params, {"use_sim_time": True}],
            )
        ],
    )

    # ekf_global_params = os.path.join(
    #     get_package_share_directory(package_name),
    #     "config",
    #     "ekf_global_params.yaml",
    # )

    # # Delay EKF until after controllers are up (period=4.0) so the robot exists
    # # and /diff_cont/odom + /imu/data are already publishing before EKF initialises.
    # # Starting EKF too early causes a bad initial state that never recovers cleanly.
    # ekf_global_node = TimerAction(
    #     period=6.0,
    #     actions=[
    #         Node(
    #             package="robot_localization",
    #             executable="ekf_node",
    #             name="ekf_global",
    #             output="screen",
    #             parameters=[ekf_global_params, {"use_sim_time": True}],
    #             remappings=[
    #                 ("odometry/filtered", "/odometry/global"),
    #             ],
    #         )
    #     ],
    # )

    apriltag_config = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "apriltag.yaml",
    )

    apriltag_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag_node",
                output="screen",
                remappings=[
                    ("image_rect", "/depth_camera_rear/image_raw"),
                    ("camera_info", "/depth_camera_rear/camera_info"),
                ],
                parameters=[apriltag_config, {"use_sim_time": True}],
            )
        ],
    )

    apriltag_map_odom = TimerAction(
        period=8.0,
        actions=[
            Node(
                package=package_name,
                executable="apriltag_map_odom_3d",
                name="apriltag_map_odom_3d",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    # apriltag_tag_base = TimerAction(
    #     period=9.0,
    #     actions=[
    #         Node(
    #             package=package_name,
    #             executable="apriltag_tag_base",
    #             name="apriltag_tag_base",
    #             output="screen",
    #             parameters=[{"use_sim_time": True}],
    #         )
    #     ],
    # )

    return LaunchDescription([
        gazebo_model_path,
        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        controller_spawners,
        ekf_local_node,
        apriltag_node,
        apriltag_map_odom
    ])