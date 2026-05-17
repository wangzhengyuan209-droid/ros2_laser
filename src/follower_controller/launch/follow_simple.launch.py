import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 相机驱动（在独立的 ascam_ros2_ws 里，用绝对路径找）
    ascamera_launch = os.path.expanduser(
        '~/robot_ws/src/ascam_ros2_ws/src/ascamera/launch/hp60c.launch.py'
    )

    return LaunchDescription([

        # 1. 深度相机驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ascamera_launch),
        ),

        # 2. 静态 TF（相机安装位置，根据实际调整 xyz）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link'],
        ),

        # 3. 人物检测（MediaPipe Holistic）
        Node(
            package='person_tracker',
            executable='person_tracker_node.py',
            name='person_tracker',
            output='screen',
            parameters=[{
                'depth_scale': 0.001,
                'fall_tilt_threshold': 50.0,
                'detection_confidence': 0.6,
                'tracking_confidence': 0.5,
                'debug_window': True,
            }],
        ),

        # 4. 底盘控制（/cmd_vel → STM32 → 电机）
        Node(
            package='base_controller',
            executable='base_controller_node',
            name='base_controller',
            output='screen',
        ),

        # 5. 跟随控制器
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
