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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('qpl_rover')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    nav_autonomy = LaunchConfiguration('nav_autonomy')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # Controller and recoveries -> /cmd_vel_nav_raw -> velocity_smoother -> /cmd_vel_nav -> drive mux
    common_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', '/cmd_vel_nav_raw'),
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
                bringup_dir,
                'behavior_trees',
                'navigate_to_pose_w_replanning_and_recovery.xml'
            ),
            description='Full path to the behavior tree xml file to use'
        ),

        DeclareLaunchArgument(
            'nav_autonomy',
            default_value='true',
            description='Also start the Nav2 autonomy node (idle until a basestation button or qpl_auto)'
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
                # Humble's name for it; default_bt_xml_filename was Foxy's and is ignored.
                {'default_nav_to_pose_bt_xml': default_bt_xml_filename},
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
                ('cmd_vel', 'cmd_vel_nav_raw'),
                ('cmd_vel_smoothed', 'cmd_vel_nav'),
            ],
        ),

        # Depth cloud -> pre-classified obstacle/clearing clouds for the costmaps.
        *[
            Node(
                package='qpl_rover',
                executable='depth_obstacle_filter',
                name=f'depth_obstacle_filter_{direction}',
                output='screen',
                parameters=[
                    os.path.join(bringup_dir, 'config', 'depth_obstacle_filter.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=common_remappings,
            )
            for direction in ('front', 'rear')
        ],

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

        # Nav2 autonomy (NAV_EXCAVATE / NAV_DEPOSIT / AUTO). Only starts the node; nothing moves
        # until a command arrives on /nav_autonomy/command. The blind autonomy node is separate.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('qpl_autonomy'), 'launch', 'nav', 'nav_autonomy.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(nav_autonomy),
        ),
    ])