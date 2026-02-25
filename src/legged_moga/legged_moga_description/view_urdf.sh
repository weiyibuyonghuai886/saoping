#!/bin/bash

# RViz URDF查看脚本
# 用于快速启动RViz查看moga机器人模型

echo "正在启动RViz查看moga机器人模型..."
echo "================================"

# 加载ROS2环境
source /opt/ros/humble/setup.bash

# 获取当前工作空间的路径（假设脚本在 src/legged_moga/legged_moga_description/ 下）
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$( cd "$SCRIPT_DIR/../../../.." && pwd )"

# 加载工作空间
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
    echo "已加载工作空间: $WORKSPACE_ROOT"
else
    echo "警告: 未找到工作空间安装文件，请先编译工作空间"
    echo "运行: cd $WORKSPACE_ROOT && colcon build --packages-select legged_moga_description"
fi

# 启动launch文件（使用包名而非绝对路径）
ros2 launch legged_moga_description display.launch.py

echo "================================"
echo "RViz已关闭"

