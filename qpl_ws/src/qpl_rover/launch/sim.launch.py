from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path, environ
import sys

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# rover_cameras' launch helpers live beside its launch files
sys.path.append(path.join(get_package_share_directory("rover_cameras"), "launch"))
from launch_utils import gazebo_stream_encoders  # noqa: E402

rover_pkg: str = get_package_share_directory("qpl_rover")
gazebo_directory: str = get_package_share_directory("gazebo_ros")

def setup_gazebo(context):
    """ Separate function to evaluate headless option. """
    headless = LaunchConfiguration('headless').perform(context)
    mode = "gzserver" if headless == "true" else "gazebo"
    world_path = path.join(rover_pkg, "worlds", LaunchConfiguration('world').perform(context))
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
        PythonLaunchDescriptionSource(
            path.join(rover_pkg, "launch", "components.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_vslam": LaunchConfiguration("use_vslam").perform(context),
        }.items()
    )

    return [components]

def make_relay(name: str, source: str, destination: str, lazy: bool = False):
    return Node(
        package="topic_tools",
        executable="relay",
        name=name,
        arguments=[source, destination],
        parameters=[{"lazy": lazy}],
    )

def make_camera_relays(camera_name, depth_image_name):
    """depth_image_name matches the real camera's depth topic, so depth_obstacle_filter
    reads the same names in sim and on the rover."""
    return [
        make_relay(
            f"relay_{camera_name}_color",
            f"/{camera_name}/image_raw",
            f"/{camera_name}/color/image_raw"
        ),
        make_relay(
            f"relay_{camera_name}_info",
            f"/{camera_name}/camera_info",
            f"/{camera_name}/color/camera_info"
        ),
        make_relay(
            f"relay_{camera_name}_compressed_color",
            f"/{camera_name}/image_raw/compressed",
            f"/{camera_name}/color/image_raw/compressed"
        ),
        make_relay(
            f"relay_{camera_name}_depth_image",
            f"/{camera_name}_depth/depth/image_raw",  # 480p depth sensor publishes under its own camera_name
            f"/{camera_name}/depth/{depth_image_name}"
        ),
        make_relay(
            f"relay_{camera_name}_depth_info",
            f"/{camera_name}_depth/camera_info",  # same intrinsics as the depth image
            f"/{camera_name}/depth/camera_info"
        ),
        # 10-13 MB a frame and nothing on the rover side needs it, so only relayed
        # while something (e.g. RViz) subscribes.
        make_relay(
            f"relay_{camera_name}_depth_points",
            f"/{camera_name}_depth/points",
            f"/{camera_name}/depth/points",
            lazy=True,
        ),
    ]

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

    world_parameter = DeclareLaunchArgument(
        'world',
        default_value='us_arena_terrain.world',
        description='World file in qpl_rover/worlds. us_arena.world is the old flat floor.'
    )

    use_vslam_parameter = DeclareLaunchArgument(
        'use_vslam',
        default_value='true',
        description='Run RGB-D visual odometry and fuse it into the local EKF as odom1. Set false to disable.'
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
                    "-x", "3.45", # set to 0 (default) if not using april_arena.world
                    "-y", "1.13", # set to 0 (default) if not using april_arena.world
                    "-z", "0.35"], # drop in from above so it never spawns inside the terrain
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        headless_parameter,
        run_components_parameter,
        use_vslam_parameter,
        world_parameter,
        gazebo_model_path,
        rsp,
        OpaqueFunction(function=setup_gazebo),
        spawn_entity,
        OpaqueFunction(function=setup_components),
        *make_camera_relays("depth_camera_front", "image_raw"),        # Orbbec
        *make_camera_relays("depth_camera_rear", "image_rect_raw"),    # RealSense
        *gazebo_stream_encoders(),
    ])
