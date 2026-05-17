#!/bin/bash
# 一键启动：相机 + 人物追踪 + 跟随控制
source ~/robot_ws/src/ascam_ros2_ws/install/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch follower_controller follow_simple.launch.py
