# Custom Controller Messages

这个包定义了用于机器人控制的自定义消息类型。

## 消息定义

### CustomControlCommand.msg

用于发送机器人控制命令的消息类型，包含32个关节的控制信息：
- 12个下肢关节
- 2个下肢占位关节
- 14个上肢关节
- 2个腰部关节
- 2个头部关节

#### 字段说明

- `header`: 标准ROS消息头
- `joint_names`: 关节名称数组（32个元素）
- `position`: 位置命令数组（弧度，32个元素）
- `velocity`: 速度命令数组（rad/s，32个元素）
- `effort`: 力矩命令数组（Nm，32个元素）
- `kp`: 位置增益数组（32个元素）
- `kd`: 速度增益数组（32个元素）

## 编译

首先编译消息包：

```bash
cd /home/bridge/deploy/moga_ros2_deploy
colcon build --packages-select custom_controller_msgs
source install/setup.bash
```

然后编译legged_system包：

```bash
colcon build --packages-select legged_system
source install/setup.bash
```

## 使用示例

发布测试命令：

```bash
ros2 topic pub --once /robot_controller/commands custom_controller_msgs/msg/CustomControlCommand "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
joint_names: [leg_joint_1, leg_joint_2, leg_joint_3, leg_joint_4, leg_joint_5, leg_joint_6, leg_joint_7, leg_joint_8, leg_joint_9, leg_joint_10, leg_joint_11, leg_joint_12, leg_joint_13, leg_joint_14, arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5, arm_joint_6, arm_joint_7, arm_joint_8, arm_joint_9, arm_joint_10, arm_joint_11, arm_joint_12, arm_joint_13, arm_joint_14, waist_yaw, waist_pitch, neck_yaw, neck_pitch]
position: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
velocity: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
effort: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
kp: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
kd: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
"
```

## 关节映射

### 实际关节到消息关节的映射关系

| 消息关节名称 | 索引 | 实际关节名称 | 实际索引 | 说明 |
|-------------|------|-------------|---------|------|
| leg_joint_1 | 0 | leg_l1_joint | 0 | 左髋俯仰 |
| leg_joint_2 | 1 | leg_l2_joint | 1 | 左髋横滚 |
| leg_joint_3 | 2 | leg_l3_joint | 2 | 左髋偏航 |
| leg_joint_4 | 3 | leg_l4_joint | 3 | 左膝 |
| leg_joint_5 | 4 | leg_l5_joint | 4 | 左踝俯仰 |
| leg_joint_6 | 5 | leg_l6_joint | 5 | 左踝横滚 |
| leg_joint_7 | 6 | leg_r1_joint | 6 | 右髋俯仰 |
| leg_joint_8 | 7 | leg_r2_joint | 7 | 右髋横滚 |
| leg_joint_9 | 8 | leg_r3_joint | 8 | 右髋偏航 |
| leg_joint_10 | 9 | leg_r4_joint | 9 | 右膝 |
| leg_joint_11 | 10 | leg_r5_joint | 10 | 右踝俯仰 |
| leg_joint_12 | 11 | leg_r6_joint | 11 | 右踝横滚 |
| leg_joint_13 | 12 | - | - | 占位（保留） |
| leg_joint_14 | 13 | - | - | 占位（保留） |
| arm_joint_1 | 14 | left_shoulder_pitch_joint | 13 | 左肩俯仰 |
| arm_joint_2 | 15 | left_shoulder_roll_joint | 14 | 左肩横滚 |
| arm_joint_3 | 16 | left_shoulder_yaw_joint | 15 | 左肩偏航 |
| arm_joint_4 | 17 | left_elbow_joint | 16 | 左肘 |
| arm_joint_5 | 18 | left_wrist_roll_joint | 17 | 左腕横滚 |
| arm_joint_6 | 19 | left_wrist_pitch_joint | 18 | 左腕俯仰 |
| arm_joint_7 | 20 | left_wrist_yaw_joint | 19 | 左腕偏航 |
| arm_joint_8 | 21 | right_shoulder_pitch_joint | 20 | 右肩俯仰 |
| arm_joint_9 | 22 | right_shoulder_roll_joint | 21 | 右肩横滚 |
| arm_joint_10 | 23 | right_shoulder_yaw_joint | 22 | 右肩偏航 |
| arm_joint_11 | 24 | right_elbow_joint | 23 | 右肘 |
| arm_joint_12 | 25 | right_wrist_roll_joint | 24 | 右腕横滚 |
| arm_joint_13 | 26 | right_wrist_pitch_joint | 25 | 右腕俯仰 |
| arm_joint_14 | 27 | right_wrist_yaw_joint | 26 | 右腕偏航 |
| waist_yaw | 28 | - | - | 占位（保留） |
| waist_pitch | 29 | torso_joint | 12 | 躯干 |
| neck_yaw | 30 | - | - | 占位（保留） |
| neck_pitch | 31 | - | - | 占位（保留） |

## 话题信息

### 控制命令话题

- **话题名称**: `/robot_controller/commands`
- **消息类型**: `custom_controller_msgs/msg/CustomControlCommand`
- **发布位置**: `LeggedSystemHardware::write()`函数
- **发布频率**: 与控制循环频率一致
- **队列大小**: 10

### 关节状态反馈话题

- **话题名称**: `/robot_controller/joint_states`
- **消息类型**: `sensor_msgs/msg/JointState`
- **发布位置**: `LeggedSystemHardware::read()`函数
- **发布频率**: 与控制循环频率一致
- **队列大小**: 10

查看关节状态反馈：

```bash
ros2 topic echo /robot_controller/joint_states
```

查看话题信息：

```bash
ros2 topic info /robot_controller/joint_states
ros2 topic hz /robot_controller/joint_states
```

### 关节状态反馈消息示例

```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: 'base_link'
name: 
  - leg_joint_1
  - leg_joint_2
  - ...
  - neck_pitch
position: [0.0, 0.0, ..., 0.0]  # 32个关节的位置 (rad)
velocity: [0.0, 0.0, ..., 0.0]  # 32个关节的速度 (rad/s)
effort: [0.0, 0.0, ..., 0.0]    # 32个关节的力矩 (Nm)
```

## ⚠️ 重要说明

### 话题变更

本版本已**完全移除**与实际电机硬件的通信：
- ❌ **已删除**: `/xlab/hr/low_cmd` (interaction_msgs::msg::LowCommand) - 电机控制
- ❌ **已删除**: `/xlab/hr/low_state` (interaction_msgs::msg::LowState) - 电机状态
- ✅ **新增**: `/robot_controller/commands` (CustomControlCommand) - 控制命令（发布）
- ✅ **新增**: `/robot_controller/joint_states` (JointState) - 状态反馈（发布）

### ⚠️ 硬件通信警告

**当前版本不与实际电机硬件通信！**
- 发布的 `/robot_controller/joint_states` 数据全部为0（模拟数据）
- 如需实际硬件控制，需要创建桥接节点或恢复原有代码

### 注意事项

1. 所有数组长度必须为32
2. 占位关节的值会被设置为0
3. 关节名称顺序固定，不可更改
4. 控制命令消息由`LeggedSystemHardware::write()`函数自动发布
5. 关节状态反馈消息由`LeggedSystemHardware::read()`函数自动发布
6. 两个话题的关节名称和顺序完全一致，便于对应
7. ⚠️ 系统不再发布/订阅原有的电机通信话题（`/xlab/hr/low_cmd` 和 `/xlab/hr/low_state`）
8. ⚠️ **关节状态数据全部为0，不反映实际硬件状态**

