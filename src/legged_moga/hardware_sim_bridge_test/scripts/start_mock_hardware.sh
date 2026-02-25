#!/bin/bash
# 启动模拟硬件端 (Domain 1)

cd /home/bridge/deploy/moga_ros2_deploy
source install/setup.bash
export ROS_DOMAIN_ID=1

echo "========================================="
echo "启动模拟硬件端 (Domain ID: 1)"
echo "========================================="

ros2 launch hardware_sim_bridge_test mock_hardware.launch.py

