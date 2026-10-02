"""Browser teleop HUD: a single-window alternative to RViz for driving.

Starts teleop_hud (basestation/hud/hud_node.py), which serves the page on
http://localhost:<port> and then opens it in the default browser. Monitoring
only: nothing here publishes a command; drive with the controller as usual.

Cameras: on the rover the feeds arrive as h264 over the teleop link
(transport:=ffmpeg). Browsers can't take those packets directly, so two
image_transport republishers decode them to JPEG locally on this laptop; that
costs laptop CPU but no extra link bandwidth. In sim Gazebo already publishes
JPEG (transport:=compressed) and the HUD subscribes to it directly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FRONT_BASE = "/depth_camera_front/color/image_raw"
REAR_BASE = "/depth_camera_rear/color/image_raw"


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("true", "1", "yes")
    transport = LaunchConfiguration("transport").perform(context)
    port = int(LaunchConfiguration("port").perform(context))
    host = LaunchConfiguration("host").perform(context)
    open_browser = LaunchConfiguration("open_browser").perform(context).lower() in ("true", "1", "yes")

    actions = []
    if transport == "compressed":
        front, rear = f"{FRONT_BASE}/compressed", f"{REAR_BASE}/compressed"
    else:
        front, rear = "/hud/front_cam/compressed", "/hud/rear_cam/compressed"

        def decoder(name, base, out):
            return Node(
                package="image_transport",
                executable="republish",
                name=name,
                arguments=[transport, "compressed"],
                remappings=[(f"in/{transport}", f"{base}/{transport}"),
                            ("out/compressed", f"{out}/compressed")],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )

        actions += [
            decoder("hud_front_decoder", FRONT_BASE, "/hud/front_cam"),
            decoder("hud_rear_decoder", REAR_BASE, "/hud/rear_cam"),
        ]

    actions.append(Node(
        package="basestation",
        executable="teleop_hud",
        name="teleop_hud",
        parameters=[{
            "use_sim_time": use_sim_time,
            "port": port,
            "host": host,
            "front_camera_topic": front,
            "rear_camera_topic": rear,
        }],
        output="screen",
    ))

    if open_browser:
        # Give the server a moment to bind before the browser asks for it.
        actions.append(TimerAction(period=2.0, actions=[ExecuteProcess(
            cmd=["xdg-open", f"http://localhost:{port}"], output="log")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation time if true."),
        DeclareLaunchArgument("transport", default_value="ffmpeg",
                              description="Camera transport arriving from the rover: 'ffmpeg' on the "
                                          "rover, 'compressed' in sim (Gazebo does not publish ffmpeg)."),
        DeclareLaunchArgument("port", default_value="8765", description="HTTP port for the HUD page."),
        DeclareLaunchArgument("host", default_value="127.0.0.1",
                              description="Bind address. 0.0.0.0 lets other machines on the network "
                                          "open the HUD too (read-only, but anyone on the LAN can view)."),
        DeclareLaunchArgument("open_browser", default_value="true",
                              description="Open the HUD in the default browser once it is up."),
        OpaqueFunction(function=launch_setup),
    ])
