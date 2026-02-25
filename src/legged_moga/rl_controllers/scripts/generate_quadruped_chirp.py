#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成四足/多关节的 Chirp 扫频测试 CSV 文件，方便用已有的绘图脚本可视化。

用法:
    python3 generate_quadruped_chirp.py --joints 12 --f0 0.1 --f1 10 --duration 30

输出每个电机/关节一个 CSV，列格式适配 `plot_chirp_test.py`：
Time,Target_Position,Current_Position,Current_Velocity,Torque_Command,Torque_Current,Kp,Kd

默认会把 CSV 写到 `../test_data/`（相对于脚本路径）。
"""

import argparse
from pathlib import Path
import numpy as np
import pandas as pd


def linear_chirp(t, f0, f1, T, amp=1.0, phase=0.0, offset=0.0):
    # 线性频率扫频：相位 phi(t) = 2*pi*(f0*t + 0.5*k*t^2), k = (f1-f0)/T
    k = (f1 - f0) / T
    phi = 2.0 * np.pi * (f0 * t + 0.5 * k * t * t) + phase
    return offset + amp * np.sin(phi)


def generate_for_joint(joint_idx, t, f0, f1, T, amp, offset, kp, kd, dt):
    target = linear_chirp(t, f0, f1, T, amp=amp, offset=offset)
    # 目标速度（解析导数）
    k = (f1 - f0) / T
    # d/dt sin(phi) = cos(phi) * phi_dot, phi_dot = 2*pi*(f0 + k*t)
    phi_dot = 2.0 * np.pi * (f0 + k * t)
    phi = 2.0 * np.pi * (f0 * t + 0.5 * k * t * t)
    target_vel = amp * np.cos(phi) * phi_dot

    # 简单一阶低通模拟实际位置跟踪（time constant tau）
    tau = 0.05  # 响应时间常数，可调整
    alpha = dt / (tau + dt)
    current = np.zeros_like(target)
    current_vel = np.zeros_like(target_vel)

    for i in range(1, len(t)):
        current[i] = current[i-1] + alpha * (target[i-1] - current[i-1])
        current_vel[i] = (current[i] - current[i-1]) / dt

    # 控制器输出（简化）： torque_cmd = kp * pos_error + kd * vel_error
    pos_error = target - current
    vel_error = target_vel - current_vel
    torque_cmd = kp * pos_error + kd * vel_error

    # 模拟实际扭矩为命令扭矩加一些噪声
    torque_current = torque_cmd + np.random.normal(scale=0.02 * np.maximum(1.0, np.abs(torque_cmd)), size=torque_cmd.shape)

    # Kp/Kd 列（常数）
    kp_col = np.full_like(t, kp)
    kd_col = np.full_like(t, kd)

    df = pd.DataFrame({
        'Time': t,
        'Target_Position': target,
        'Current_Position': current,
        'Current_Velocity': current_vel,
        'Torque_Command': torque_cmd,
        'Torque_Current': torque_current,
        'Kp': kp_col,
        'Kd': kd_col,
    })

    return df


def main():
    parser = argparse.ArgumentParser(description='生成多关节 Chirp 扫频 CSV 用于绘图')
    parser.add_argument('--joints', type=int, default=12, help='关节/电机数 (默认12)')
    parser.add_argument('--f0', type=float, default=0.1, help='起始频率 Hz')
    parser.add_argument('--f1', type=float, default=10.0, help='结束频率 Hz')
    parser.add_argument('--duration', type=float, default=30.0, help='测试时长 秒')
    parser.add_argument('--amp', type=float, default=0.4, help='角度振幅 (rad)')
    parser.add_argument('--offset', type=float, default=0.0, help='位置偏置 (rad)')
    parser.add_argument('--kp', type=float, default=120.0, help='P 增益')
    parser.add_argument('--kd', type=float, default=4.0, help='D 增益')
    parser.add_argument('--hz', type=float, default=200.0, help='采样频率 Hz')
    parser.add_argument('--outdir', type=str, default=None, help='输出目录，默认 ../test_data')

    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    outdir = Path(args.outdir) if args.outdir else script_dir.parent / 'test_data'
    outdir.mkdir(parents=True, exist_ok=True)

    dt = 1.0 / args.hz
    t = np.arange(0.0, args.duration + 1e-9, dt)

    print(f'生成 {args.joints} 个关节的 Chirp 数据: {args.f0}Hz -> {args.f1}Hz, {args.duration}s, {args.hz}Hz 采样')

    for j in range(1, args.joints + 1):
        df = generate_for_joint(j, t, args.f0, args.f1, args.duration, args.amp, args.offset, args.kp, args.kd, dt)

        fname = f'motor_{j}_response_chirp_f{args.f0:.2f}to{args.f1:.2f}Hz_{args.duration:.2f}s_amp{args.amp:.2f}_kp{args.kp:.2f}_kd{args.kd:.2f}.csv'
        outpath = outdir / fname
        df.to_csv(outpath, index=False)
        print(f'写入: {outpath}')

    print('完成。你可以用 plot_chirp_test.py 或 plot_latest_chirp.py 进行可视化。')


if __name__ == '__main__':
    main()
