from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path, environ

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

selected_world = "arena_april.world"
rover_pkg: str = get_package_share_directory("qpl_rover")
gazebo_directory: str = get_package_share_directory("gazebo_ros")

def setup_gazebo(context):
    """ Separate function to evaluate headless option. """
    headless = LaunchConfiguration('headless').perform(context)
    mode = "gzserver" if headless == "true" else "gazebo"
    world_path = path.join(rover_pkg, "worlds", selected_world)
    gazebo_params_file = path.join(rover_pkg, "config", "gazebo_params.yaml")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(gazebo_directory, "launch", f"{mode}.launch.py")),
        launch_arguments={
            "world": world_path,
            "extra_gazebo_args": f"--ros-args --params-file {gazebo_params_file}"
        }.items()
    )

    return [gazebo]

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
    headless_parameter = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to run Gazebo in headless mode (true won\'t show GUI).'
    )

    run_components_parameter = DeclareLaunchArgument(
        'run_components',
        default_value='true',
        description='Whether to run the simulation with components.'
    )

    # tell gazebo where to find the apriltag model so the texture loads on any machine
    models_path = path.join(rover_pkg, "worlds")
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_path + ':' + environ.get('GAZEBO_MODEL_PATH', '')
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(rover_pkg, "launch", f"rsp.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "use_ros2_control": "true"
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

    return LaunchDescription([
        headless_parameter,
        run_components_parameter,
        gazebo_model_path,
        rsp,
        OpaqueFunction(function=setup_gazebo),
        spawn_entity,
        OpaqueFunction(function=setup_components),
    ])
