"""Laser battle robot launch file.

Usage:
  # With red union (attack enemy color + red/hit enemies)
  ros2 launch laser_battle laser_battle.launch.py team_color:=green
  ros2 launch laser_battle laser_battle.launch.py team_color:=blue

  # Without red union (attack enemy color only)
  ros2 launch laser_battle laser_battle.launch.py team_color:=green include_red:=false
  ros2 launch laser_battle laser_battle.launch.py team_color:=blue include_red:=false

  # Without dodge (add enable_dodge:=false)
  ros2 launch laser_battle laser_battle.launch.py team_color:=green enable_dodge:=false
  ros2 launch laser_battle laser_battle.launch.py team_color:=blue enable_dodge:=false
  ros2 launch laser_battle laser_battle.launch.py team_color:=green include_red:=false enable_dodge:=false
  ros2 launch laser_battle laser_battle.launch.py team_color:=blue include_red:=false enable_dodge:=false
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
    ascamera_launch = os.path.join(
        get_package_share_directory('ascamera'), 'launch', 'hp60c.launch.py'
    )

    enable_camera = LaunchConfiguration('enable_camera')
    show_camera = LaunchConfiguration('show_camera')
    show_debug = LaunchConfiguration('show_debug')
    enable_enemy_detector = LaunchConfiguration('enable_enemy_detector')
    enable_attack = LaunchConfiguration('enable_attack')
    team_color = LaunchConfiguration('team_color')
    include_red = LaunchConfiguration('include_red')
    enable_dodge = LaunchConfiguration('enable_dodge')

    return LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('show_camera', default_value='false'),
        DeclareLaunchArgument('show_debug', default_value='true'),
        DeclareLaunchArgument('enable_enemy_detector', default_value='true'),
        DeclareLaunchArgument('enable_attack', default_value='true'),
        DeclareLaunchArgument('team_color', default_value='green'),
        DeclareLaunchArgument('include_red', default_value='true'),
        DeclareLaunchArgument('enable_dodge', default_value='true'),

        # Camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ascamera_launch),
            condition=IfCondition(enable_camera),
        ),

        # Camera viewer
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='camera_view',
            output='screen',
            arguments=['/ascamera_hp60c/camera_publisher/rgb0/image'],
            condition=IfCondition(show_camera),
        ),

        # Debug image viewer
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='debug_view',
            output='screen',
            arguments=['/enemy_debug_image'],
            condition=IfCondition(show_debug),
        ),

        # Base controller
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
                'angular_speed': 0.9,
                'speed_step': 0.25,
            }],
            prefix='xterm -e',
        ),

        # Enemy detector
        Node(
            package='laser_battle',
            executable='enemy_detector',
            name='enemy_detector',
            output='screen',
            parameters=[{
                'team_color': team_color,
                'include_red': include_red,
                'min_area': 300,
                'max_area': 300000,
                'depth_range': 500,
            }],
            condition=IfCondition(enable_enemy_detector),
        ),

        # Attack controller
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
                'enable_dodge': enable_dodge,
            }],
            condition=IfCondition(enable_attack),
        ),
    ])
