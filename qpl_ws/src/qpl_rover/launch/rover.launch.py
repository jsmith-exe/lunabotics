from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path, environ

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


rover_pkg: str = get_package_share_directory("qpl_rover")

def setup_components(context):
    """ Separate function to evaluate whether to components. """
    run_components = LaunchConfiguration('run_components').perform(context)
    if run_components != "true":
        return []

    components = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(rover_pkg, "launch", "components.launch.py"))
    )
    return [components]

def generate_launch_description():

    package_name = "qpl_rover"

    run_components_parameter = DeclareLaunchArgument(
        'run_components',
        default_value='true',
        description='Whether to run the simulation with components.'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(rover_pkg, "launch", f"rsp.launch.py")),
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