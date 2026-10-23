"""Browser teleop HUD: a single-window alternative to RViz for driving.

Starts teleop_hud (basestation/hud/hud_node.py), which serves the page on
http://localhost:<port> and then opens it in the default browser. Monitoring
only: nothing here publishes a command; drive with the controller as usual.

Cameras: the HUD watches only the h264 teleop streams that RViz uses,
<camera>/color/teleop_stream/ffmpeg, made by rover_cameras' stream encoder on the
rover and in sim alike. Browsers can't take those packets directly, so teleop_hud
runs an image_transport republisher per camera (hud_front_decoder,
hud_rear_decoder) that decodes them to JPEG on this laptop: laptop CPU, no extra
link bandwidth. Switching a camera off in the HUD stops its decoder, and with it
the stream over the link.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    def flag(name):
        return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")

    port = int(LaunchConfiguration("port").perform(context))
    host = LaunchConfiguration("host").perform(context)

    actions = [Node(
        package="basestation",
        executable="teleop_hud",
        name="teleop_hud",
        parameters=[{
            "use_sim_time": flag("use_sim_time"),
            "port": port,
            "host": host,
            "telemetry": flag("telemetry"),
            "costmap": flag("costmap"),
            "front_camera": flag("front_camera"),
            "rear_camera": flag("rear_camera"),
            "link_interface": LaunchConfiguration("link_interface").perform(context),
        }],
        output="screen",
    )]

    if flag("open_browser"):
        # Give the server a moment to bind before the browser asks for it.
        actions.append(TimerAction(period=2.0, actions=[ExecuteProcess(
            cmd=["xdg-open", f"http://localhost:{port}"], output="log")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation time if true."),
        DeclareLaunchArgument("port", default_value="8765", description="HTTP port for the HUD page."),
        DeclareLaunchArgument("host", default_value="127.0.0.1",
                              description="Bind address. 0.0.0.0 lets other machines on the network "
                                          "open the HUD too (read-only, but anyone on the LAN can view)."),
        DeclareLaunchArgument("telemetry", default_value="true",
                              description="Start with per-topic rates and the /rosout log on. Off drops "
                                          "the /rosout subscription, which can be heavy over the link. "
                                          "Toggle it from the page's SYSTEM drawer."),
        DeclareLaunchArgument("costmap", default_value="true",
                              description="Start with the global costmap on the map. Off drops the "
                                          "subscription (~0.6 Mbit/s with Nav2 running). Toggle with K."),
        DeclareLaunchArgument("front_camera", default_value="true",
                              description="Start with the front camera stream on. Toggle with 1."),
        DeclareLaunchArgument("rear_camera", default_value="true",
                              description="Start with the rear camera stream on. Toggle with 2."),
        DeclareLaunchArgument("link_interface", default_value="",
                              description="Network interface the link meter watches. Empty: the one "
                                          "with the default route."),
        DeclareLaunchArgument("open_browser", default_value="true",
                              description="Open the HUD in the default browser once it is up."),
        OpaqueFunction(function=launch_setup),
    ])
