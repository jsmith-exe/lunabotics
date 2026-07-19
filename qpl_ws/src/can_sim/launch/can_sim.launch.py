import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

FAKE_USB = "/tmp/fake_can_rx"
CAN_USB = "/dev/ttyUSB0"

def delay(process, period):
    return TimerAction(actions=[process], period=period)

def generate_launch_description():
    # Create fake USB
    socat_process = ExecuteProcess(
        cmd=["sudo", "socat", f"PTY,link={CAN_USB},rawer", f"PTY,link={FAKE_USB},rawer"],
        output="screen",
    )

    # Grants fake USB read write perms, regardless of user
    chmod_fake_usb_process = ExecuteProcess(
        cmd=["sudo", "chmod", "666", FAKE_USB],
        output="screen",
    )
    chmod_can_usb_process = ExecuteProcess(
        cmd=["sudo", "chmod", "666", CAN_USB],
        output="screen",
    )

    can_sim_node = Node(
        package="can_sim",
        executable="can_sim",
        name="can_sim",
        output="screen",
    )

    return LaunchDescription([
        socat_process,
        delay(chmod_fake_usb_process, 1.0),
        delay(chmod_can_usb_process, 1.0),
        delay(can_sim_node, 2.0),
    ])