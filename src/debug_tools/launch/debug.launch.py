from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='debug_tools',
            executable='hsv_sampler',
            name='hsv_sampler',
            output='screen',
        ),
        Node(
            package='debug_tools',
            executable='attack_monitor',
            name='attack_monitor',
            output='screen',
        ),
    ])
