#!/bin/bash
# 启动 Domain Bridge (Domain 1)

cd /home/bridge/deploy/moga_ros2_deploy
source install/setup.bash
export ROS_DOMAIN_ID=1

echo "========================================="
echo "启动 Domain Bridge (从 Domain 1 到 Domain 0)"
echo "========================================="

ros2 launch hardware_sim_bridge_test domain_bridge.launch.py

