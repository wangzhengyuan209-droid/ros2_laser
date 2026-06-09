from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_controller',
            executable='base_controller_node',
            name='base_controller',
            output='screen',
        ),
    ])
