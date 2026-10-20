"""Browser teleop HUD: a single-window alternative to RViz for driving.

Starts teleop_hud (basestation/hud/hud_node.py), which serves the page on
http://localhost:<port> and then opens it in the default browser. Monitoring
only: nothing here publishes a command; drive with the controller as usual.

Cameras: the HUD watches the same h264 teleop streams as RViz,
<camera>/color/teleop_stream/ffmpeg, made by rover_cameras' stream encoder on the
rover and in sim alike. Browsers can't take those packets directly, so two
image_transport republishers decode them to JPEG locally on this laptop; that
costs laptop CPU but no extra link bandwidth. transport:=compressed skips the
decoders and takes <camera>/color/image_raw/compressed instead (hud_demo, or as a
fallback when the stream encoder isn't running).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FRONT_STREAM = "/depth_camera_front/color/teleop_stream"
REAR_STREAM = "/depth_camera_rear/color/teleop_stream"
FRONT_RAW = "/depth_camera_front/color/image_raw"
REAR_RAW = "/depth_camera_rear/color/image_raw"


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("true", "1", "yes")
    transport = LaunchConfiguration("transport").perform(context)
    port = int(LaunchConfiguration("port").perform(context))
    host = LaunchConfiguration("host").perform(context)
    open_browser = LaunchConfiguration("open_browser").perform(context).lower() in ("true", "1", "yes")
    telemetry = LaunchConfiguration("telemetry").perform(context).lower() in ("true", "1", "yes")

    actions = []
    if transport == "compressed":
        front, rear = f"{FRONT_RAW}/compressed", f"{REAR_RAW}/compressed"
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
                parameters=[{
                    "use_sim_time": use_sim_time,
                    # Software h264 decoding, as in rviz.launch.py: the default h264_cuvid
                    # holds several frames before showing one and has turned the picture
                    # green. The parameter takes the un-remapped base topic name, "in".
                    f"in.{transport}.decoders.h264": "h264",
                }],
                output="screen",
            )

        actions += [
            decoder("hud_front_decoder", FRONT_STREAM, "/hud/front_cam"),
            decoder("hud_rear_decoder", REAR_STREAM, "/hud/rear_cam"),
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
            "telemetry": telemetry,
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
                              description="'ffmpeg': decode <camera>/color/teleop_stream/ffmpeg (rover and "
                                          "sim). 'compressed': take <camera>/color/image_raw/compressed "
                                          "directly (hud_demo, or if the stream encoder isn't running)."),
        DeclareLaunchArgument("port", default_value="8765", description="HTTP port for the HUD page."),
        DeclareLaunchArgument("host", default_value="127.0.0.1",
                              description="Bind address. 0.0.0.0 lets other machines on the network "
                                          "open the HUD too (read-only, but anyone on the LAN can view)."),
        DeclareLaunchArgument("telemetry", default_value="true",
                              description="Start with per-topic rates and the /rosout log on. Off drops "
                                          "the /rosout subscription, which can be heavy over the link. "
                                          "Toggle it from the page's SYSTEM drawer."),
        DeclareLaunchArgument("open_browser", default_value="true",
                              description="Open the HUD in the default browser once it is up."),
        OpaqueFunction(function=launch_setup),
    ])
