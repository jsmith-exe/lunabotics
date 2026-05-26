from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from os import path

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
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

    return LaunchDescription([
        run_components_parameter,
        rsp,
        OpaqueFunction(function=setup_components),
    ])