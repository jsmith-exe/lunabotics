from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    example_command = ExecuteProcess(
        cmd=["socat", "PTY,link=/dev/ttyUSB0,rawer", "PTY,link=/tmp/fake_can_rx,rawer"],
        output="screen",
    )

    example_node = Node(
        package="can_sim",
        executable="can_sim",
        name="can_sim",
        output="screen",
    )

    return LaunchDescription([
        example_command,
        example_node,
    ])
