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

# random craters/boulders generator lives beside the world files
sys.path.append(path.join(rover_pkg, "worlds"))
import arena_obstacles  # noqa: E402

OBSTACLE_WORLD = "us_arena_terrain.world"  # the only world arena_obstacles knows how to fill
gazebo_directory: str = get_package_share_directory("gazebo_ros")

def setup_gazebo(context):
    """ Separate function to evaluate headless option. """
    headless = LaunchConfiguration('headless').perform(context)
    mode = "gzserver" if headless == "true" else "gazebo"
    world = LaunchConfiguration('world').perform(context)
    world_path = path.join(rover_pkg, "worlds", world)
    if world == OBSTACLE_WORLD and LaunchConfiguration('obstacles').perform(context) == "true":
        seed = int(LaunchConfiguration('obstacle_seed').perform(context))
        world_path, seed = arena_obstacles.build_world(seed)
        print(f"[sim] random craters/boulders, seed {seed} (obstacle_seed:={seed} repeats this layout), "
              f"layout in {path.join(path.dirname(world_path), 'obstacles.yaml')}")
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

    obstacles_parameter = DeclareLaunchArgument(
        'obstacles',
        default_value='true',
        description=f'Add random craters (carved into the floor) and boulders to {OBSTACLE_WORLD}, '
                    'placed per the rulebook and different every launch. false gives the plain floor.'
    )

    obstacle_seed_parameter = DeclareLaunchArgument(
        'obstacle_seed',
        default_value='-1',
        description='Seed for the obstacle layout; -1 picks a new one each launch (it is printed).'
    )

    use_vslam_parameter = DeclareLaunchArgument(
        'use_vslam',
        default_value='true',
        description='Run RGB-D visual odometry and fuse it into the local EKF as odom1. Set false to disable.'
    )

    # tell gazebo where to find the apriltag model so the texture loads on any machine
    models_path = path.join(rover_pkg, "worlds")
    # and where to find the URDF's package://qpl_rover/... meshes, which Gazebo looks up as
    # model://qpl_rover/... (the wheel mesh)
    packages_path = path.dirname(rover_pkg)
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_path + ':' + packages_path + ':' + environ.get('GAZEBO_MODEL_PATH', '')
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
                    "-y", "1.6", # 1.13 put the front camera 0.5 m from the tag, too close to see all of it
                    "-Y", "-1.66", # facing the AprilTag at (3.32, 0.02), so the rover is localised from the start
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
        obstacles_parameter,
        obstacle_seed_parameter,
        gazebo_model_path,
        rsp,
        OpaqueFunction(function=setup_gazebo),
        spawn_entity,
        OpaqueFunction(function=setup_components),
        *make_camera_relays("depth_camera_front", "image_rect_raw"),   # RealSense
        *make_camera_relays("depth_camera_rear", "image_raw"),         # Orbbec
        *gazebo_stream_encoders(),
    ])
