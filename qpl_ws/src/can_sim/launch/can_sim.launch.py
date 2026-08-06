"""
This will require your password for sudo, it will ask in the console, if it wasn't previously run.
It may seem to continue without letting you enter your password; enter it anyway and press enter - the processes requiring sudo are still listening (due to multithreaded output).
If it fails the first run, run it again.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


def delay(process, period):
    return TimerAction(
        period=period,
        actions=[process]
    )

# TODO avoid using sudo and delays (maybe chain everything into one command?)
# TODO remove 'magic strings' /tmp/fake_can_rx and /dev/ttyUSB0
def generate_launch_description():
    socat_process = ExecuteProcess(
        cmd=["sudo", "socat", "PTY,link=/dev/ttyUSB0,rawer", "PTY,link=/tmp/fake_can_rx,rawer"],
        output="screen",
    )

    # TODO remove if this works for others
    # user = os.environ.get("USER")
    # chown_process = ExecuteProcess(
    #     cmd=["sudo", "chown", f"{user}:{user}", "/tmp/fake_can_rx"],
    #     output="screen",
    # )

    chmod_sim_port_process = ExecuteProcess(
        cmd=["sudo", "chmod", "666", "/tmp/fake_can_rx"],
        output="screen",
    )


    chmod_devtty_port_process = ExecuteProcess(
        cmd=["sudo", "chmod", "666", "/dev/ttyUSB0"],
        output="screen",
    )

    can_sim_node = Node(package="can_sim", executable="can_sim", name="can_sim", output="screen")

    # sudo chmod 666
    return LaunchDescription([
        socat_process,
        delay(chmod_sim_port_process, 2.0),
        delay(chmod_devtty_port_process, 2.0),
        delay(can_sim_node, 4.0),
    ])
