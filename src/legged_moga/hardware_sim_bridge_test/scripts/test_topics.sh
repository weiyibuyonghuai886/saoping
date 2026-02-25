#!/bin/bash
# 测试话题通信

cd /home/bridge/deploy/moga_ros2_deploy
source install/setup.bash

echo "========================================="
echo "测试 Domain 0 (主控端)"
echo "========================================="
export ROS_DOMAIN_ID=0

echo ""
echo "--- Domain 0 话题列表 ---"
ros2 topic list

echo ""
echo "--- 查找 joint_states 相关话题 ---"
ros2 topic list | grep joint_states

echo ""
echo "--- /remote/joint_states 话题信息 ---"
ros2 topic info /remote/joint_states

echo ""
echo "--- /remote/joint_states 话题频率 (5秒) ---"
timeout 5 ros2 topic hz /remote/joint_states

echo ""
echo "========================================="
echo "测试 Domain 1 (硬件模拟端)"
echo "========================================="
export ROS_DOMAIN_ID=1

echo ""
echo "--- Domain 1 话题列表 ---"
ros2 topic list

echo ""
echo "--- 查找 joint_states 相关话题 ---"
ros2 topic list | grep joint_states

echo ""
echo "--- /joint_states 话题信息 ---"
ros2 topic info /joint_states

echo ""
echo "========================================="
echo "测试完成"
echo "========================================="

