#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Chirp扫频测试数据可视化脚本

用法:
    python3 plot_chirp_test.py <csv_file_path>
    
示例:
    python3 plot_chirp_test.py ../test_data/motor_4_response_chirp_f0.10to10.00Hz_30.00s_0.50to1.30rad_kp120.00_kd4.00.csv
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys
import os
from pathlib import Path

# 设置matplotlib支持中文显示
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号


def plot_chirp_test(csv_file):
    """
    绘制chirp扫频测试数据
    
    参数:
        csv_file: CSV文件路径
    """
    # 检查文件是否存在
    if not os.path.exists(csv_file):
        print(f"错误：文件不存在 - {csv_file}")
        sys.exit(1)
    
    # 读取CSV数据
    print(f"正在读取数据: {csv_file}")
    df = pd.read_csv(csv_file)
    
    # 提取数据
    time = df['Time'].values
    target_pos = df['Target_Position'].values
    current_pos = df['Current_Position'].values
    current_vel = df['Current_Velocity'].values
    torque_cmd = df['Torque_Command'].values
    torque_current = df['Torque_Current'].values
    kp = df['Kp'].values[0]  # 假设KP固定
    kd = df['Kd'].values[0]  # 假设KD固定
    
    # 计算位置误差
    pos_error = target_pos - current_pos
    
    # 从文件名提取参数
    filename = Path(csv_file).stem
    parts = filename.split('_')
    motor_id = parts[1]
    
    # 创建图形
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(f'Motor {motor_id} Chirp扫频测试结果 (Kp={kp}, Kd={kd})', 
                 fontsize=16, fontweight='bold')
    
    # ========== 子图1: 位置跟踪 ==========
    ax1 = plt.subplot(3, 2, 1)
    ax1.plot(time, target_pos, 'r--', linewidth=2, label='目标位置', alpha=0.8)
    ax1.plot(time, current_pos, 'b-', linewidth=1.5, label='实际位置', alpha=0.9)
    ax1.set_xlabel('时间 (s)', fontsize=11)
    ax1.set_ylabel('位置 (rad)', fontsize=11)
    ax1.set_title('位置跟踪曲线', fontsize=12, fontweight='bold')
    ax1.legend(loc='best', fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    # ========== 子图2: 位置误差 ==========
    ax2 = plt.subplot(3, 2, 2)
    ax2.plot(time, pos_error, 'g-', linewidth=1.5, label='位置误差')
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=1, alpha=0.3)
    ax2.set_xlabel('时间 (s)', fontsize=11)
    ax2.set_ylabel('误差 (rad)', fontsize=11)
    ax2.set_title('位置跟踪误差', fontsize=12, fontweight='bold')
    ax2.legend(loc='best', fontsize=10)
    ax2.grid(True, alpha=0.3)
    
    # 计算误差统计
    error_mean = np.mean(np.abs(pos_error))
    error_max = np.max(np.abs(pos_error))
    error_std = np.std(pos_error)
    ax2.text(0.02, 0.98, f'平均误差: {error_mean:.4f} rad\n'
                          f'最大误差: {error_max:.4f} rad\n'
                          f'标准差: {error_std:.4f} rad',
             transform=ax2.transAxes, fontsize=9,
             verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # ========== 子图3: 速度曲线 ==========
    ax3 = plt.subplot(3, 2, 3)
    ax3.plot(time, current_vel, 'purple', linewidth=1.5, label='当前速度')
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=1, alpha=0.3)
    ax3.set_xlabel('时间 (s)', fontsize=11)
    ax3.set_ylabel('速度 (rad/s)', fontsize=11)
    ax3.set_title('关节速度曲线', fontsize=12, fontweight='bold')
    ax3.legend(loc='best', fontsize=10)
    ax3.grid(True, alpha=0.3)
    
    # ========== 子图4: 扭矩对比 ==========
    ax4 = plt.subplot(3, 2, 4)
    ax4.plot(time, torque_cmd, 'r--', linewidth=2, label='命令扭矩', alpha=0.7)
    ax4.plot(time, torque_current, 'b-', linewidth=1.5, label='实际扭矩', alpha=0.9)
    ax4.axhline(y=0, color='k', linestyle='--', linewidth=1, alpha=0.3)
    ax4.set_xlabel('时间 (s)', fontsize=11)
    ax4.set_ylabel('扭矩 (N·m)', fontsize=11)
    ax4.set_title('扭矩对比曲线', fontsize=12, fontweight='bold')
    ax4.legend(loc='best', fontsize=10)
    ax4.grid(True, alpha=0.3)
    
    # ========== 子图5: 位置-速度相图 ==========
    ax5 = plt.subplot(3, 2, 5)
    scatter = ax5.scatter(current_pos, current_vel, c=time, cmap='viridis', 
                         s=10, alpha=0.6)
    ax5.set_xlabel('位置 (rad)', fontsize=11)
    ax5.set_ylabel('速度 (rad/s)', fontsize=11)
    ax5.set_title('位置-速度相图', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=ax5)
    cbar.set_label('时间 (s)', fontsize=10)
    
    # ========== 子图6: 扭矩误差 ==========
    ax6 = plt.subplot(3, 2, 6)
    torque_error = torque_cmd - torque_current
    ax6.plot(time, torque_error, 'orange', linewidth=1.5, label='扭矩误差')
    ax6.axhline(y=0, color='k', linestyle='--', linewidth=1, alpha=0.3)
    ax6.set_xlabel('时间 (s)', fontsize=11)
    ax6.set_ylabel('扭矩误差 (N·m)', fontsize=11)
    ax6.set_title('扭矩跟踪误差', fontsize=12, fontweight='bold')
    ax6.legend(loc='best', fontsize=10)
    ax6.grid(True, alpha=0.3)
    
    # 计算扭矩误差统计
    torque_error_mean = np.mean(np.abs(torque_error))
    torque_error_max = np.max(np.abs(torque_error))
    ax6.text(0.02, 0.98, f'平均误差: {torque_error_mean:.2f} N·m\n'
                          f'最大误差: {torque_error_max:.2f} N·m',
             transform=ax6.transAxes, fontsize=9,
             verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 调整子图间距
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    
    # 保存图片
    output_file = csv_file.replace('.csv', '_plot.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"图片已保存: {output_file}")
    
    # 显示图形
    plt.show()
    
    # 打印数据统计
    print("\n========== 数据统计 ==========")
    print(f"数据点数: {len(time)}")
    print(f"测试时长: {time[-1]:.2f} 秒")
    print(f"采样频率: {len(time) / time[-1]:.2f} Hz")
    print(f"\n位置范围: [{np.min(target_pos):.4f}, {np.max(target_pos):.4f}] rad")
    print(f"速度范围: [{np.min(current_vel):.4f}, {np.max(current_vel):.4f}] rad/s")
    print(f"扭矩范围: [{np.min(torque_current):.4f}, {np.max(torque_current):.4f}] N·m")
    print(f"\n位置跟踪误差:")
    print(f"  平均: {error_mean:.4f} rad")
    print(f"  最大: {error_max:.4f} rad")
    print(f"  标准差: {error_std:.4f} rad")
    print(f"\n扭矩跟踪误差:")
    print(f"  平均: {torque_error_mean:.2f} N·m")
    print(f"  最大: {torque_error_max:.2f} N·m")
    print("=" * 30)


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print("用法: python3 plot_chirp_test.py <csv_file_path>")
        print("\n示例:")
        print("  python3 plot_chirp_test.py motor_4_response_chirp_f0.10to10.00Hz_30.00s_0.50to1.30rad_kp120.00_kd4.00.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    plot_chirp_test(csv_file)


if __name__ == "__main__":
    main()

