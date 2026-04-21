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

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), "launch", "controllers.launch.py"
        )])
    )

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), "config", "gazebo_params.yaml"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
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

    odom_localisation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), "launch", "odom_localisation.launch.py"
        )])
    )

    map_localisation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), "launch", "map_localisation.launch.py"
        )])
    )

    return LaunchDescription([
        gazebo_model_path,
        rsp,
        controllers,
        gazebo,
        spawn_entity,
        odom_localisation,
        map_localisation,
    ])