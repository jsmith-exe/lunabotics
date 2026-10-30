from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path, environ
import re
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
ros_gz_sim_directory: str = get_package_share_directory("ros_gz_sim")

def setup_gazebo(context):
    """ Separate function to evaluate headless option. """
    headless = LaunchConfiguration('headless').perform(context)
    world = LaunchConfiguration('world').perform(context)
    world_path = path.join(rover_pkg, "worlds", world)
    if world == OBSTACLE_WORLD and LaunchConfiguration('obstacles').perform(context) == "true":
        seed = int(LaunchConfiguration('obstacle_seed').perform(context))
        world_path, seed = arena_obstacles.build_world(seed)
        print(f"[sim] random craters/boulders, seed {seed} (obstacle_seed:={seed} repeats this layout), "
              f"layout in {path.join(path.dirname(world_path), 'obstacles.yaml')}")
    # -r starts unpaused; -s is the server only, rendering the cameras off-screen
    verbosity = LaunchConfiguration('gz_verbosity').perform(context)
    gz_args = f"-r -v {verbosity} " + ("-s --headless-rendering " if headless == "true" else "") + world_path

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(ros_gz_sim_directory, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "true",
        }.items()
    )

    # Gazebo's root /clock didn't publish when tested (headless server), so bridge this
    # world's own clock instead. Without it every use_sim_time node (EKF, Nav2, ...) sits
    # at time zero.
    with open(world_path) as fh:
        world_name = re.search(r'<world\s+name\s*=\s*"([^"]+)"', fh.read()).group(1)
    world_clock = f"/world/{world_name}/clock"
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_clock_bridge",
        arguments=[f"{world_clock}@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        remappings=[(world_clock, "/clock")],
        output="screen",
    )

    return [gazebo, clock_bridge]

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
        # No depth point cloud: Gazebo Harmonic publishes it in the camera body frame
        # (x forward) but stamps it with the optical frame, and nothing on the rover
        # side reads it. depth_obstacle_filter works from the depth image.
    ]

def make_gz_bridges():
    """Gazebo Harmonic -> ROS topics, named as gazebo_ros used to publish them."""
    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            parameters=[{
                "config_file": path.join(rover_pkg, "config", "gz_bridge.yaml"),
                "use_sim_time": True,
            }],
            output="screen",
        ),
        # image_transport, so <camera>/image_raw/compressed exists like it did with gazebo_ros
        Node(
            package="ros_gz_image",
            executable="image_bridge",
            name="gz_image_bridge",
            arguments=["depth_camera_front/image_raw", "depth_camera_rear/image_raw"],
            remappings=[
                ("depth_camera_front/image_raw", "/depth_camera_front/image_raw"),
                ("depth_camera_rear/image_raw", "/depth_camera_rear/image_raw"),
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
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
        description='World file in qpl_rover/worlds. Only us_arena_terrain.world is set up for '
                    'Gazebo Harmonic and the sand plugin; the other worlds are the old Gazebo Classic ones.'
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

    gz_verbosity_parameter = DeclareLaunchArgument(
        'gz_verbosity',
        default_value='2',
        description='Gazebo log level, 0-4. 3 also shows plugin info, e.g. the terramechanics plugin loading.'
    )

    use_vslam_parameter = DeclareLaunchArgument(
        'use_vslam',
        default_value='true',
        description='Run RGB-D visual odometry and fuse it into the local EKF as odom1. Set false to disable.'
    )

    # tell gazebo where to find the apriltag and terrain models so they load on any machine
    models_path = path.join(rover_pkg, "worlds")
    # and where to find the URDF's package://qpl_rover/... meshes, which Gazebo looks up as
    # model://qpl_rover/... (the wheel mesh)
    packages_path = path.dirname(rover_pkg)
    gazebo_model_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path + ':' + packages_path + ':' + environ.get('GZ_SIM_RESOURCE_PATH', '')
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
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-topic", "robot_description",
                    "-name", "rover",
                    "-x", "3.45", # set to 0 (default) if not using april_arena.world
                    "-y", "1.13", # set to 0 (default) if not using april_arena.world
                    "-z", "0.35"], # drop in from above so it never spawns inside the terrain
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        headless_parameter,
        gz_verbosity_parameter,
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
        *make_gz_bridges(),
        *gazebo_stream_encoders(),
    ])
