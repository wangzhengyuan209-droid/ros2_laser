import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('base_controller')
    nav2_params = os.path.join(pkg_dir, 'param', 'nav2_params.yaml')

    base_controller_node = Node(
        package='base_controller',
        executable='base_controller_node',
        name='base_controller',
        output='screen',
        parameters=[nav2_params],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        base_controller_node
    ])