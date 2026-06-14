"""Laser battle robot launch file.

Launches: camera + base_controller + keyboard_control + enemy_detector + attack_controller

Usage:
  ros2 launch laser_battle laser_battle.launch.py
  ros2 launch laser_battle laser_battle.launch.py show_camera:=false
  ros2 launch laser_battle laser_battle.launch.py show_debug:=true
  ros2 launch laser_battle laser_battle.launch.py enable_attack:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Camera driver
    ascamera_launch = os.path.join(
        get_package_share_directory('ascamera'), 'launch', 'hp60c.launch.py'
    )

    # Launch arguments
    enable_camera = LaunchConfiguration('enable_camera')
    show_camera = LaunchConfiguration('show_camera')
    show_debug = LaunchConfiguration('show_debug')
    enable_enemy_detector = LaunchConfiguration('enable_enemy_detector')
    enable_attack = LaunchConfiguration('enable_attack')

    return LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('show_camera', default_value='true'),
        DeclareLaunchArgument('show_debug', default_value='false'),
        DeclareLaunchArgument('enable_enemy_detector', default_value='true'),
        DeclareLaunchArgument('enable_attack', default_value='true'),

        # Camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ascamera_launch),
            condition=IfCondition(enable_camera),
        ),

        # Camera viewer (rqt_image_view - raw feed)
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='camera_view',
            output='screen',
            arguments=['/ascamera_hp60c/camera_publisher/rgb0/image'],
            condition=IfCondition(show_camera),
        ),

        # Debug image viewer (target detection overlay)
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='debug_view',
            output='screen',
            arguments=['/enemy_debug_image'],
            condition=IfCondition(show_debug),
        ),

        # Base controller (STM32 motor bridge)
        Node(
            package='base_controller',
            executable='base_controller_node',
            name='base_controller',
            output='screen',
        ),

        # Keyboard control
        Node(
            package='laser_battle',
            executable='keyboard_control',
            name='keyboard_control',
            output='screen',
            parameters=[{
                'linear_speed': 0.3,
                'angular_speed': 0.7,
                'speed_step': 0.1,
            }],
            prefix='xterm -e',
        ),

        # Enemy detector (color-based target tracking)
        Node(
            package='laser_battle',
            executable='enemy_detector',
            name='enemy_detector',
            output='screen',
            parameters=[{
                'lock_radius': 30,
                'min_area': 300,
                'max_area': 300000,
            }],
            condition=IfCondition(enable_enemy_detector),
        ),

        # Attack controller (PID auto-aim while moving)
        Node(
            package='laser_battle',
            executable='attack_controller',
            name='attack_controller',
            output='screen',
            parameters=[{
                'img_width': 640,
                'img_height': 480,
                'dead_zone': 25,
                'forward_speed': 0.25,
                'kp': 0.004,
                'ki': 0.0001,
                'kd': 0.002,
                'max_angular': 1.5,
            }],
            condition=IfCondition(enable_attack),
        ),
    ])
