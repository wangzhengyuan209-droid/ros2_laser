from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follower_controller',
            executable='follower_controller_node.py',
            name='follower_controller',
            output='screen',
            parameters=[{
                'follow_distance': 1.5,
                'approach_distance': 0.3,
                'kp_dist': 0.5,
                'kp_angle': 1.0,
                'max_linear': 0.5,
                'max_angular': 0.8,
                'obstacle_threshold': 0.3,
                'lost_timeout': 3.0,
                'control_hz': 20.0,
            }],
        ),
    ])
