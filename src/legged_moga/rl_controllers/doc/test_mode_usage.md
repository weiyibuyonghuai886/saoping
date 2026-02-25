# 电机测试模式使用说明

## 概述

电机测试模式（TEST Mode）用于对单个电机进行系统性测试，支持多种波形和参数组合，自动记录测试数据到CSV文件。

## 功能特点

- **多种波形支持**：正弦波、傅里叶合成波、三角波、阶跃信号、线性波
- **参数扫描**：自动遍历不同的幅度和频率组合
- **数据记录**：自动保存测试数据为CSV格式
- **灵活配置**：通过YAML文件配置所有测试参数

## 配置文件

测试参数在 `config/test_motor.yaml` 中配置：

```yaml
test_motor:
  motor_id: 8              # 测试电机ID（关节索引从1开始）
  kp: 40.0                 # PD控制器Kp增益
  kd: 0.1                  # PD控制器Kd增益
  start_rad: -0.3          # 起始角度（弧度）
  end_rad: 0.3             # 结束角度（弧度）
  rad_step: 0.2            # 角度步进（弧度）
  start_freq: 0.5          # 起始频率（Hz）
  end_freq: 1.1            # 结束频率（Hz）
  freq_step: 0.1           # 频率步进（Hz）
  waveform_type: "sine"    # 波形类型
  save_path: "/path/to/save/data"  # 数据保存路径
  max_test_count: 5000     # 每组测试的循环次数
  decimation: 10           # 数据采样率
```

## 波形类型

### 1. 正弦波 (sine)
```yaml
waveform_type: "sine"
```
标准正弦波，适合基础频响测试。

### 2. 傅里叶合成波 (fourier)
```yaml
waveform_type: "fourier"
```
多个频率成分叠加的复杂波形，带随机相位和幅度扰动。

### 3. 三角波 (triangle)
```yaml
waveform_type: "triangle"
```
线性上升和下降的三角波，适合测试速度响应。

### 4. 阶跃信号 (step)
```yaml
waveform_type: "step"
step_time: 5.0  # 阶跃发生的时间（秒）
```
从起始位置突变到结束位置，适合测试阶跃响应。

### 5. 线性波 (linear)
```yaml
waveform_type: "linear"
```
从0开始线性增长到目标位置。

## 使用方法

### 1. 启动系统
```bash
ros2 launch rl_controllers rl_control_real.launch.py
```

### 2. 进入测试模式
在系统未启动控制（start_control为false）时，发送switch_mode信号：
```bash
ros2 topic pub --once /switch_mode std_msgs/msg/Float32 "data: 1.0"
```
系统会从DEFAULT模式切换到TEST模式。

### 3. 退出测试模式
再次发送switch_mode信号：
```bash
ros2 topic pub --once /switch_mode std_msgs/msg/Float32 "data: 1.0"
```
系统会从TEST模式切换回DEFAULT模式。

## 测试流程

1. **参数初始化**：第一次进入TEST模式时，从参数服务器读取配置
2. **自动测试**：系统自动遍历所有频率和幅度组合
3. **数据记录**：每组测试完成后自动保存CSV文件
4. **测试完成**：所有组合测试完毕后自动切换回DEFAULT模式

## 数据输出

测试数据保存在配置的`save_path`目录下，文件名格式：
```
motor_response_{waveform}_{start}to{end}rad_{freq}Hz_kp{kp}_kd{kd}.csv
```

CSV文件包含以下列：
- Time: 时间戳（秒）
- Target_Position: 目标位置（弧度）
- Current_Position: 当前位置（弧度）
- Torque_Command: 计划扭矩（Nm）
- Torque_Current: 实际扭矩（Nm）

## 示例配置

### 快速测试配置
```yaml
test_motor:
  motor_id: 8
  kp: 30.0
  kd: 1.0
  start_rad: -0.1
  end_rad: 0.1
  rad_step: 0.1
  start_freq: 0.5
  end_freq: 1.0
  freq_step: 0.5
  waveform_type: "sine"
  max_test_count: 1000
```

### 详细频响测试
```yaml
test_motor:
  motor_id: 8
  kp: 40.0
  kd: 0.1
  start_rad: -0.3
  end_rad: 0.3
  rad_step: 0.1
  start_freq: 0.1
  end_freq: 2.0
  freq_step: 0.1
  waveform_type: "sine"
  max_test_count: 5000
```

## 注意事项

1. **安全**：测试前确保机器人处于安全位置，测试电机可以自由运动
2. **路径**：确保保存路径存在且有写入权限
3. **参数**：根据实际电机特性调整KP/KD增益
4. **幅度**：注意角度范围不要超出关节限位
5. **测试时长**：测试时长 = (频率步数) × (幅度步数) × (max_test_count) × (控制周期)

## 故障排除

### 文件保存失败
- 检查保存路径是否存在
- 检查是否有写入权限
- 确保磁盘空间充足

### 电机不动作
- 检查motor_id是否正确（从1开始计数）
- 检查是否已进入TEST模式
- 检查KP/KD参数是否合理

### 数据异常
- 调整decimation参数改变采样率
- 检查max_test_count是否足够
- 验证波形参数设置是否正确

