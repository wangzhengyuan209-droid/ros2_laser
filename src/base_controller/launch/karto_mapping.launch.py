import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 获取包路径（复用你现有工程的路径）
    pkg_dir = get_package_share_directory('base_controller')
    
    # 2. 声明参数（匹配你现有工程的配置）
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')

    # 3. slam_toolbox 核心节点（异步建图，适合实时性场景）
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',  # 异步建图（推荐），同步用sync_slam_toolbox_node
        name='slam_toolbox',
        output='screen',
        parameters=[
            # 加载slam_toolbox参数文件（下一步创建）
            os.path.join(pkg_dir, 'param', 'slam_toolbox_params.yaml'),
            {'use_sim_time': use_sim_time}  # 覆盖参数文件的sim_time配置
        ],
        remappings=[
            ('/scan', scan_topic),          # 映射激光话题（匹配你现有/scan）
            ('/odom', odom_topic)           # 映射里程计话题（匹配你现有/odom）
        ]
    )

    # 4. RViz2节点（复用你现有RViz配置，无需修改）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(pkg_dir, 'rviz', 'karto_slam.rviz')  # 复用原有RViz配置
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. 启动描述
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', 
                              default_value='false', 
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('scan_topic', 
                              default_value='/scan', 
                              description='Laser scan topic name'),
        DeclareLaunchArgument('odom_topic', 
                              default_value='/odom', 
                              description='Odometry topic name'),
        slam_toolbox_node,
        rviz_node
    ])