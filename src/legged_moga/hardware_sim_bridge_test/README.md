# Hardware Simulation Bridge Test

这个包用于测试 ROS 2 Domain ID 隔离通信方案。

## 架构说明

### Domain 0 (主控端)
- 运行 `rl_control_real.launch.py`
- 运行 RL 控制器
- 接收来自 Domain 1 的 `/remote/joint_states` 话题

### Domain 1 (硬件模拟端)
- 运行 `mock_hardware.launch.py` - 模拟硬件系统
- 发布 `/joint_states` 话题
- 运行 `domain_bridge.launch.py` - 桥接话题到 Domain 0

### Domain Bridge
- 单向桥接：Domain 1 → Domain 0
- 话题重命名：`/joint_states` → `/remote/joint_states`

## 使用方法

### 1. 构建包

```bash
cd /home/bridge/deploy/moga_ros2_deploy
colcon build --packages-select hardware_sim_bridge_test
source install/setup.bash
```

### 2. 启动测试

需要打开 **3-4 个终端**：

#### 终端 1: 启动模拟硬件端 (Domain 1)
```bash
cd /home/bridge/deploy/moga_ros2_deploy
source install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch hardware_sim_bridge_test mock_hardware.launch.py
```

#### 终端 2: 启动 Domain Bridge (Domain 1)
```bash
cd /home/bridge/deploy/moga_ros2_deploy
source install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch hardware_sim_bridge_test domain_bridge.launch.py
```

#### 终端 3: 启动主控端 (Domain 0)
```bash
cd /home/bridge/deploy/moga_ros2_deploy
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch rl_controllers rl_control_real.launch.py
```

#### 终端 4: 验证话题通信 (Domain 0)
```bash
cd /home/bridge/deploy/moga_ros2_deploy
source install/setup.bash
export ROS_DOMAIN_ID=0

# 列出所有话题
ros2 topic list

# 查看 /remote/joint_states 话题
ros2 topic echo /remote/joint_states

# 查看话题频率
ros2 topic hz /remote/joint_states

# 查看话题信息
ros2 topic info /remote/joint_states
```

### 3. 验证通信隔离

验证 Domain 0 看不到 Domain 1 的原始 `/joint_states`：

```bash
# 在 Domain 0
export ROS_DOMAIN_ID=0
ros2 topic list | grep joint_states
# 应该只看到 /remote/joint_states，看不到 /joint_states

# 在 Domain 1
export ROS_DOMAIN_ID=1
ros2 topic list | grep joint_states
# 应该看到 /joint_states
```

## 测试检查清单

- [ ] Domain 1 的 mock 硬件系统成功启动
- [ ] Domain 1 的 `/joint_states` 正常发布
- [ ] domain_bridge 成功启动
- [ ] Domain 0 能接收到 `/remote/joint_states`
- [ ] Domain 0 看不到原始的 `/joint_states` (隔离成功)
- [ ] `/remote/joint_states` 的数据与 Domain 1 的 `/joint_states` 一致
- [ ] 主控端 rl_controller 可以正常运行

## 故障排查

### domain_bridge 无法启动
- 确保安装了 `domain_bridge` 包：`sudo apt install ros-$ROS_DISTRO-domain-bridge`
- 检查 Domain ID 设置是否正确

### 看不到 /remote/joint_states
- 确认 domain_bridge 正在运行
- 确认两个终端的 ROS_DOMAIN_ID 设置正确
- 检查 domain_bridge 的日志输出

### Mock 硬件无法启动
- 确保安装了 mock_components：`sudo apt install ros-$ROS_DISTRO-ros2-control-test-assets`
- 检查 URDF/xacro 文件语法是否正确

## 配置文件说明

- `config/mock_hardware.xacro` - 模拟硬件的机器人描述文件
- `config/mock_controllers.yaml` - controller_manager 配置
- `config/domain_bridge.yaml` - domain_bridge 桥接配置
- `launch/mock_hardware.launch.py` - 启动模拟硬件
- `launch/domain_bridge.launch.py` - 启动域桥接

