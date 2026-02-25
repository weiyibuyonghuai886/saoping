# 快速启动指南

## 一、准备工作

### 1. 安装依赖
```bash
sudo apt install ros-humble-domain-bridge
```

### 2. 构建包
```bash
cd /home/bridge/deploy/moga_ros2_deploy
colcon build --packages-select hardware_sim_bridge_test --symlink-install
source install/setup.bash
```

## 二、启动测试

### 方法 1: 使用便捷脚本（推荐）

需要打开 **3 个终端**：

#### 终端 1: 模拟硬件端
```bash
cd /home/bridge/deploy/moga_ros2_deploy/src/legged_moga/hardware_sim_bridge_test/scripts
./start_mock_hardware.sh
```

#### 终端 2: Domain Bridge
```bash
cd /home/bridge/deploy/moga_ros2_deploy/src/legged_moga/hardware_sim_bridge_test/scripts
./start_domain_bridge.sh
```

#### 终端 3: 测试验证
```bash
cd /home/bridge/deploy/moga_ros2_deploy/src/legged_moga/hardware_sim_bridge_test/scripts
./test_topics.sh
```

### 方法 2: 手动启动

#### 终端 1: 启动模拟硬件 (Domain 1)
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

#### 终端 3: 验证通信 (Domain 0)
```bash
cd /home/bridge/deploy/moga_ros2_deploy
source install/setup.bash
export ROS_DOMAIN_ID=0

# 查看话题列表
ros2 topic list

# 查看 /remote/joint_states 数据
ros2 topic echo /remote/joint_states

# 查看话题频率
ros2 topic hz /remote/joint_states
```

## 三、验证清单

运行后验证以下项：

- [ ] 终端 1 显示 controller_manager 和 joint_state_broadcaster 成功启动
- [ ] 终端 2 显示 domain_bridge 成功启动（无错误）
- [ ] 在 Domain 0 能看到 `/remote/joint_states` 话题
- [ ] 在 Domain 0 看不到 `/joint_states` 话题（隔离成功）
- [ ] `/remote/joint_states` 以约 100Hz 频率发布
- [ ] 数据包含所有 30 个关节

## 四、常见问题

### Q1: domain_bridge 无法启动
**A**: 确保已安装：
```bash
sudo apt install ros-humble-domain-bridge
```

### Q2: 看不到 /remote/joint_states
**A**: 
1. 确认 domain_bridge 正在运行且无错误
2. 确认 ROS_DOMAIN_ID 设置正确（Domain 0）
3. 等待几秒钟让桥接建立

### Q3: 如何停止所有进程
**A**:
```bash
# 停止所有相关进程
pkill -f mock_hardware
pkill -f domain_bridge
pkill -f ros2_control_node
```

## 五、目录结构

```
hardware_sim_bridge_test/
├── config/
│   ├── mock_hardware.xacro      # 模拟硬件机器人描述
│   ├── mock_controllers.yaml    # Controller Manager 配置
│   └── domain_bridge.yaml       # Domain Bridge 配置
├── launch/
│   ├── mock_hardware.launch.py  # 启动模拟硬件
│   └── domain_bridge.launch.py  # 启动域桥接
├── scripts/
│   ├── start_mock_hardware.sh   # 便捷脚本：启动硬件
│   ├── start_domain_bridge.sh   # 便捷脚本：启动桥接
│   └── test_topics.sh           # 便捷脚本：测试验证
├── README.md                     # 详细说明文档
├── QUICK_START.md               # 本文件
└── TEST_REPORT.md               # 测试报告
```

## 六、下一步

详细的测试报告和架构说明请查看：
- [README.md](README.md) - 完整使用说明
- [TEST_REPORT.md](TEST_REPORT.md) - 详细测试报告

