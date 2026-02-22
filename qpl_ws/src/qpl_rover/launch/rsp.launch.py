import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _clean_urdf_xml(xml: str) -> str:
    # Parse and re-serialize only <robot> (drops XML header + comments)
    root = ET.fromstring(xml)
    if root.tag != "robot":
        robot = root.find("robot")
        if robot is None:
            raise RuntimeError("Could not find <robot> element in generated URDF.")
        root = robot

    urdf = ET.tostring(root, encoding="unicode", method="xml")
    # single-line for max compatibility
    return " ".join(urdf.split())


def _make_rsp(context, *args, **kwargs):
    package = "qpl_rover"
    pkg_share = get_package_share_directory(package)
    xacro_file = os.path.join(pkg_share, "description", "rover.urdf.xacro")

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_ros2_control = LaunchConfiguration("use_ros2_control").perform(context)

    # xacro CLI style args -> xacro mappings dict
    mappings = {
        "use_ros2_control": use_ros2_control,
        "sim_mode": use_sim_time,
    }

    doc = xacro.process_file(xacro_file, mappings=mappings)
    xml = doc.toxml()
    robot_description = _clean_urdf_xml(xml)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": (use_sim_time.lower() == "true"),
            }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use sim time if true",
        ),
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="true",
            description="Use ros2_control if true",
        ),
        OpaqueFunction(function=_make_rsp),
    ])