#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动查找并绘制最新的Chirp扫频测试数据

用法:
    python3 plot_latest_chirp.py
    
或者指定test_data目录:
    python3 plot_latest_chirp.py /path/to/test_data
"""

import os
import sys
import glob
from pathlib import Path

# 导入绘图脚本
import plot_chirp_test


def find_latest_chirp_csv(test_data_dir=None):
    """
    查找最新的chirp测试CSV文件
    
    参数:
        test_data_dir: test_data目录路径，如果为None则自动搜索
    
    返回:
        最新的chirp CSV文件路径，如果没有找到则返回None
    """
    # 如果没有指定目录，尝试自动查找
    if test_data_dir is None:
        # 获取脚本所在目录
        script_dir = Path(__file__).parent.absolute()
        
        # 可能的test_data位置
        possible_paths = [
            script_dir.parent / "test_data",  # ../test_data
            script_dir.parent.parent.parent.parent / "install/rl_controllers/share/rl_controllers/test_data",  # install目录
            Path.cwd() / "test_data",  # 当前目录下的test_data
        ]
        
        for path in possible_paths:
            if path.exists():
                test_data_dir = path
                print(f"找到test_data目录: {test_data_dir}")
                break
    else:
        test_data_dir = Path(test_data_dir)
    
    if test_data_dir is None or not test_data_dir.exists():
        print("错误：未找到test_data目录")
        return None
    
    # 查找所有chirp CSV文件
    chirp_files = list(test_data_dir.glob("motor_*_response_chirp_*.csv"))
    
    if not chirp_files:
        print(f"错误：在 {test_data_dir} 中未找到chirp测试文件")
        return None
    
    # 按修改时间排序，获取最新的文件
    latest_file = max(chirp_files, key=lambda p: p.stat().st_mtime)
    
    print(f"找到 {len(chirp_files)} 个chirp测试文件")
    print(f"最新文件: {latest_file.name}")
    
    return str(latest_file)


def main():
    """主函数"""
    test_data_dir = None
    
    # 如果提供了参数，使用指定的目录
    if len(sys.argv) > 1:
        test_data_dir = sys.argv[1]
        print(f"使用指定的test_data目录: {test_data_dir}")
    
    # 查找最新的chirp CSV文件
    csv_file = find_latest_chirp_csv(test_data_dir)
    
    if csv_file is None:
        print("\n提示：你可以手动指定CSV文件:")
        print("  python3 plot_chirp_test.py <csv_file_path>")
        sys.exit(1)
    
    # 绘制图形
    print(f"\n正在绘制: {csv_file}\n")
    plot_chirp_test.plot_chirp_test(csv_file)


if __name__ == "__main__":
    main()

