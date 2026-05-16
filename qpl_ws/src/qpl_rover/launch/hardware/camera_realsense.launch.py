from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_params = {
        "camera.color.image_raw.ffmpeg.encoder": "libx264",
        "camera.color.image_raw.ffmpeg.bit_rate": 1000000,
        "camera.color.image_raw.ffmpeg.qmax": 51,
        "pointcloud__neon_.enable": True,
        "initial_reset": True,
    }

    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[camera_params],
            output='screen',
        )
    ])