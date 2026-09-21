# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


### costmap mask addition start ###
# The keepout mask is generated fresh on every launch from the arena selected in
# config/arena/selector.yaml, so it always matches the current arena dimensions.
MASK_RESOLUTION = 0.05  # meters per pixel (matches the costmap resolution)
MASK_PADDING_M = 1.0    # keepout border drawn around the arena, meters


def load_arena_config(bringup_dir):
    arena_config_dir = os.path.join(bringup_dir, 'config', 'arena')

    with open(os.path.join(arena_config_dir, 'selector.yaml'), 'r') as file:
        arena_name = yaml.safe_load(file)['arena']

    arena_config_path = os.path.join(arena_config_dir, f'{arena_name}.yaml')
    if not os.path.exists(arena_config_path):
        raise FileNotFoundError(
            f"Unknown arena '{arena_name}'. "
            f"Expected configuration at: {arena_config_path}"
        )

    with open(arena_config_path, 'r') as file:
        return arena_name, yaml.safe_load(file)['arena']


def generate_keepout_mask(arena_width_x, arena_length_y, output_dir):
    """Write keepout_mask.pgm/.yaml: arena interior free (255), everything else keepout (0).

    Map (0, 0) is the bottom-left corner of the arena, so the image origin sits
    at negative padding.
    """
    pad_px = int(round(MASK_PADDING_M / MASK_RESOLUTION))
    # round(), not int(): e.g. 9.14 / 0.05 = 182.8 px and truncating would shave
    # a few cm off the far wall.
    arena_px_x = int(round(arena_width_x / MASK_RESOLUTION))
    arena_px_y = int(round(arena_length_y / MASK_RESOLUTION))
    pix_x = arena_px_x + 2 * pad_px
    pix_y = arena_px_y + 2 * pad_px

    # PGM rows run top (max y) to bottom (min y); the padding is symmetric so
    # the free rows are the same either way.
    border_row = bytes(pix_x)
    arena_row = bytes(pad_px) + b'\xff' * arena_px_x + bytes(pad_px)
    rows = [border_row] * pad_px + [arena_row] * arena_px_y + [border_row] * pad_px

    image_path = os.path.join(output_dir, 'keepout_mask.pgm')
    with open(image_path, 'wb') as f:
        f.write(f'P5\n{pix_x} {pix_y}\n255\n'.encode())
        f.write(b''.join(rows))

    yaml_path = os.path.join(output_dir, 'keepout_mask.yaml')
    with open(yaml_path, 'w') as f:
        f.write(
            f'image: {image_path}\n'
            f'resolution: {MASK_RESOLUTION}\n'
            f'origin: [{-MASK_PADDING_M:.3f}, {-MASK_PADDING_M:.3f}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n'
        )

    return yaml_path


def launch_filter_mask_server(context, bringup_dir, use_sim_time):
    arena_name, arena_config = load_arena_config(bringup_dir)
    output_dir = tempfile.mkdtemp(prefix='qpl_keepout_mask_')
    mask_yaml_file = generate_keepout_mask(
        arena_config['width'], arena_config['length'], output_dir)

    print(
        f"Generated keepout mask for arena '{arena_name}' "
        f"({arena_config['width']} x {arena_config['length']} m): {mask_yaml_file}"
    )

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': mask_yaml_file},
            ],
            remappings=[
                ('/map', '/keepout_filter_mask'),
            ],
        ),
    ]
### costmap mask addition end ###


def generate_launch_description():
    bringup_dir = get_package_share_directory('qpl_rover')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'filter_mask_server', # costmap mask addition
        'costmap_filter_info_server', # costmap mask addition
    ]

    common_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', '/cmd_vel_nav'),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav_params.yaml'),
            description='Full path to the ROS2 parameters file to use'
        ),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_w_replanning_and_recovery.xml'
            ),
            description='Full path to the behavior tree xml file to use'
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=common_remappings,
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=common_remappings,
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=common_remappings,
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
                {'default_bt_xml_filename': default_bt_xml_filename},
            ],
            remappings=common_remappings,
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=common_remappings,
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),

        ### costmap mask addition start ###
        OpaqueFunction(
            function=launch_filter_mask_server,
            args=[bringup_dir, use_sim_time],
        ),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
        ### costmap mask addition end ###

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': lifecycle_nodes},
            ],
        ),
    ])