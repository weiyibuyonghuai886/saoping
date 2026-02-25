# legged_moga_description 构建指南

## 概述
本 package 包含 MOGA 机器人的 URDF 描述文件、3D 模型、配置文件和启动文件。

## 文件结构
```
legged_moga_description/
├── CMakeLists.txt          # CMake 构建配置
├── package.xml             # ROS2 包配置文件
├── config/                 # 配置文件
│   └── gazebo_params.yaml
├── launch/                 # 启动文件
│   └── display.launch.py   # RViz 可视化启动文件
├── meshes/                 # 3D 网格文件
│   └── *.STL
├── urdf/                   # URDF/XACRO 文件
│   ├── common/
│   │   ├── gazebo.xacro    # Gazebo 插件配置
│   │   └── imu.xacro       # IMU 传感器配置
│   └── moga.urdf           # 机器人主 URDF 文件
├── rviz/                   # RViz 配置
│   └── moga.rviz
└── view_urdf.sh            # 快速查看脚本
```

## 构建步骤

### 1. 编译 package
```bash
cd /home/bridge/deploy/moga_ros2_deploy
colcon build --packages-select legged_moga_description
```

### 2. 加载工作空间
```bash
source /home/bridge/deploy/moga_ros2_deploy/install/setup.bash
```

### 3. 验证安装
```bash
ros2 pkg list | grep legged_moga_description
```

## 使用方法

### 方法 1: 使用 launch 文件
```bash
source /home/bridge/deploy/moga_ros2_deploy/install/setup.bash
ros2 launch legged_moga_description display.launch.py
```

### 方法 2: 使用快速启动脚本
```bash
cd /home/bridge/deploy/moga_ros2_deploy/src/legged_moga/legged_moga_description
./view_urdf.sh
```

## 主要特性

### 1. URDF/XACRO 配置
- ✅ 使用 xacro 格式增强可维护性
- ✅ 包含 IMU 传感器配置
- ✅ 完整的 Gazebo 物理参数

### 2. Gazebo 配置
- ✅ 所有 link 的摩擦系数配置
- ✅ 碰撞参数优化
- ✅ 视觉材质设置

### 3. ROS2 Control
- ✅ 28 个关节的 command/state interface
  - 腰部：2 个关节
  - 左臂：7 个关节
  - 右臂：7 个关节
  - 头部：2 个关节
  - 左腿：6 个关节
  - 右腿：6 个关节
- ✅ IMU 传感器接口（四元数、角速度、线加速度）

### 4. 相对路径配置
- ✅ launch 文件使用 `get_package_share_directory()`
- ✅ view_urdf.sh 自动检测工作空间路径
- ✅ URDF 中使用 `$(find ...)` 语法

## 关节列表

### 腰部 (Waist)
- waist_rotation
- waist_bend

### 左臂 (Left Arm)
- left_arm_shoulder1
- left_arm_shoulder2
- left_arm_shoulder3
- left_arm_elbow
- left_arm_forearm
- left_arm_wrist_swing
- left_arm_wrist_pitch

### 右臂 (Right Arm)
- right_arm_shoulder1
- right_arm_shoulder2
- right_bigarm_rotation
- right_arm_elbow
- right_arm_forearm
- right_arm_wrist_swing
- right_arm_wrist_pitch

### 头部 (Head)
- head_rotation
- head_pitch

### 左腿 (Left Leg)
- left_leg_hip1
- left_leg_hip_swing
- left_leg_hip_rotation
- left_leg_knee
- left_leg_ankle_pitch
- left_leg_ankle_swing

### 右腿 (Right Leg)
- right_leg_hip1
- right_leg_hip_swing
- right_leg_hip3
- right_leg_knee
- right_leg_ankle_pitch
- right_leg_ankle_swing

## 故障排除

### 编译错误
如果遇到编译错误，请检查：
1. 是否安装了所有依赖：`rosdep install --from-paths src --ignore-src -r -y`
2. ROS2 环境是否正确加载：`source /opt/ros/humble/setup.bash`

### 启动错误
如果启动失败，请检查：
1. 工作空间是否已编译
2. 是否 source 了 install/setup.bash
3. mesh 文件是否存在于 meshes/ 目录

### 路径问题
如果遇到路径找不到的问题：
1. 确保已经编译并安装了 package
2. 使用 `ros2 pkg prefix legged_moga_description` 检查安装路径
3. 检查 URDF 中的路径是否使用了 `$(find ...)` 语法

## 依赖项
- rclcpp
- robot_state_publisher
- rviz2
- joint_state_publisher_gui
- gazebo_ros_pkgs
- xacro

## 维护信息
- Package: legged_moga_description
- Version: 1.0.0
- License: BSD

