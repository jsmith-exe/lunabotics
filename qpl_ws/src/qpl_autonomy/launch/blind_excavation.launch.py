from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    forward_speed = LaunchConfiguration('forward_speed')
    drive_duration = LaunchConfiguration('drive_duration')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_forward_speed = DeclareLaunchArgument(
        'forward_speed',
        default_value='0.2',
        description='Forward creep speed during the dig pass (m/s)'
    )

    declare_drive_duration = DeclareLaunchArgument(
        'drive_duration',
        default_value='10.0',
        description='Seconds to drive forward while digging'
    )

    blind_excavation_node = Node(
        package='qpl_autonomy',
        executable='blind_excavation_node',
        name='blind_excavation_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'forward_speed': ParameterValue(forward_speed, value_type=float),
            'drive_duration': ParameterValue(drive_duration, value_type=float)
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_forward_speed,
        declare_drive_duration,
        blind_excavation_node
    ])
