from launch import LaunchDescription
from launch_ros.actions import Node

# Beacon position in the MAP frame — update this to match where your physical
# (or Gazebo) beacon anchor is placed in the arena.
BEACON_X = 0.055
BEACON_Y = 0.2


def generate_launch_description():
    beacon_params = {
        'use_sim_time': True,
        'beacon_x':    BEACON_X,
        'beacon_y':    BEACON_Y,
    }

    uwb_simulator = Node(
        package='qpl_rover',
        executable='uwb_simulator',
        name='uwb_simulator',
        output='screen',
        parameters=[beacon_params],
    )

    uwb_position_estimator = Node(
        package='qpl_rover',
        executable='uwb_position_estimator',
        name='uwb_position_estimator',
        output='screen',
        parameters=[beacon_params],
    )

    return LaunchDescription([
        uwb_simulator,
        uwb_position_estimator,
    ])
