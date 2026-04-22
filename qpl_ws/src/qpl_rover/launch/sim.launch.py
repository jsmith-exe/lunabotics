from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path, environ

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():
    selected_world = "arena_april.world"
    rover_directory: str = get_package_share_directory("qpl_rover")
    gazebo_directory: str = get_package_share_directory("gazebo_ros")

    components_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(rover_directory, "launch", "components.launch.py"))
    )

    # tell gazebo where to find the apriltag model so the texture loads on any machine
    models_path = path.join(rover_directory, "worlds")
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_path + ':' + environ.get('GAZEBO_MODEL_PATH', '')
    )

    world_path = path.join(rover_directory, "worlds", selected_world)
    gazebo_params_file = path.join(rover_directory, "config", "gazebo_params.yaml")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(gazebo_directory, "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": world_path,
            "extra_gazebo_args": f"--ros-args --params-file {gazebo_params_file}"
        }
    )



    return LaunchDescription([
        components_launch,
        gazebo_model_path,
        gazebo,
    ])