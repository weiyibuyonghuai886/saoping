#!/bin/bash
set -e

# rm -rf build install log

# 编译
# colcon build --symlink-install
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 激活环境
source install/setup.bash

#ROS2 跨机通信笔记本电脑配置
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # 设置ros2 的dds为cyclone dds

# 设置电机控制模式
ros2 topic pub --once /mode_controller/commands std_msgs/msg/Float64MultiArray \
"data: [5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5]"

# 电机使能
ros2 topic pub --once /controlword_controller/commands std_msgs/msg/Float64MultiArray \
"data: [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]"

# 电机去使能
ros2 topic pub --once /controlword_controller/commands std_msgs/msg/Float64MultiArray \
"data: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]"

# 运行
ros2 launch rl_controllers rl_control_real.launch.py