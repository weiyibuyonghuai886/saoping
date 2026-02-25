# Moga机器人URDF模型查看指南

## 文件说明

- `urdf/moga.urdf` - 机器人URDF模型文件
- `meshes/` - 机器人3D模型网格文件（STL格式）
- `view_urdf.sh` - 快速启动RViz查看脚本
- `display.launch.py` - ROS2 launch文件
- `moga.rviz` - RViz配置文件

## 快速使用

### 方法1：使用bash脚本（推荐）

```bash
cd /home/bridge/Downloads/moga
./view_urdf.sh
```

### 方法2：直接使用ROS2命令

```bash
source /opt/ros/humble/setup.bash
ros2 launch /home/bridge/Downloads/moga/display.launch.py
```

## 功能说明

启动后会打开两个窗口：

1. **RViz窗口**：显示3D机器人模型
   - 可以旋转、缩放、平移视角
   - 显示所有关节和连杆的TF坐标系

2. **Joint State Publisher GUI窗口**：控制关节运动
   - 使用滑块调整各个关节的角度
   - 实时查看机器人姿态变化

## 机器人关节列表

### 腰部关节
- waist_rotation - 腰部旋转
- waist_bend - 腰部弯曲

### 左臂关节
- left_arm_shoulder1/2/3 - 左肩关节
- left_arm_elbow - 左肘关节
- left_arm_forearm - 左前臂
- left_arm_wrist_swing - 左手腕摆动
- left_arm_wrist_pitch - 左手腕俯仰

### 右臂关节
- right_arm_shoulder1/2 - 右肩关节
- right_bigarm_rotation - 右大臂旋转
- right_arm_elbow - 右肘关节
- right_arm_forearm - 右前臂
- right_arm_wrist_swing - 右手腕摆动
- right_arm_wrist_pitch - 右手腕俯仰

### 头部关节
- head_rotation - 头部旋转
- head_pitch - 头部俯仰

### 左腿关节
- left_leg_hip1 - 左髋关节1
- left_leg_hip_swing - 左髋摆动
- left_leg_hip_rotation - 左髋旋转
- left_leg_knee - 左膝关节
- left_leg_ankle_pitch - 左踝俯仰
- left_leg_ankle_swing - 左踝摆动

### 右腿关节
- right_leg_hip1 - 右髋关节1
- right_leg_hip_swing - 右髋摆动
- right_leg_hip3 - 右髋关节3
- right_leg_knee - 右膝关节
- right_leg_ankle_pitch - 右踝俯仰
- right_leg_ankle_swing - 右踝摆动

## 退出程序

在终端按 `Ctrl+C` 即可关闭所有节点。

## 依赖要求

- ROS2 Humble
- rviz2
- robot_state_publisher
- joint_state_publisher_gui

如果缺少依赖，可以使用以下命令安装：

```bash
sudo apt install ros-humble-rviz2 ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
```

