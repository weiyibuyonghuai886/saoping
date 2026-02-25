#!/bin/bash
# 启动 RL 控制器 (Domain 0)

cd /home/bridge/deploy/moga_ros2_deploy
source install/setup.bash
export ROS_DOMAIN_ID=0

echo "========================================="
echo "启动 RL 控制器 (Domain ID: 0)"
echo "========================================="

ros2 launch rl_controllers rl_control_real.launch.py

