from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_tracker',
            executable='tracker_node',
            name='camera_tracker',
            output='screen',
            parameters=[{
                'camera_index': 0  # если нужна другая камера, меняешь на 1, 2 и т.д.
            }]
        )
    ])
