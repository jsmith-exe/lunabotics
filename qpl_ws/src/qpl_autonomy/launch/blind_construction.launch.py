from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    dump_duration = LaunchConfiguration('dump_duration')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_dump_duration = DeclareLaunchArgument(
        'dump_duration',
        default_value='10.0',
        description='Seconds to reverse-spin the drum to build the berm'
    )

    blind_construction_node = Node(
        package='qpl_autonomy',
        executable='blind_construction_node',
        name='blind_construction_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'dump_duration': ParameterValue(dump_duration, value_type=float)
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_dump_duration,
        blind_construction_node
    ])
