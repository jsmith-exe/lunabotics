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
        "arena.world"
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
        period=2.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", "robot_description", "-entity", "rover"],
                output="screen",
            )
        ]
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

    ekf_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "ekf_params.yaml",
    )

    # Delay EKF until after controllers are up (period=4.0) so the robot exists
    # and /diff_cont/odom + /imu/data are already publishing before EKF initialises.
    # Starting EKF too early causes a bad initial state that never recovers cleanly.
    ekf_node = TimerAction(
        period=6.0,
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

    return LaunchDescription([
        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        controller_spawners,
        ekf_node,   # last — needs robot + controllers + sensors all running first
    ])