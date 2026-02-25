#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
专门处理hihonr_leg_position.xml文件的脚本
将所有data[*]中的索引减去3
"""

import re
import os

def update_data_indices_in_file(file_path):
    """
    更新文件中的data数组索引
    将所有data[*]中的索引减去3
    """
    try:
        # 读取文件内容
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 使用正则表达式找到所有data[*]模式并更新索引
        def replace_data_index(match):
            # 提取索引数字
            index_str = match.group(1)
            try:
                index_num = int(index_str)
                new_index = index_num - 3
                return f'data[{new_index}]'
            except ValueError:
                # 如果无法转换为数字，保持原样
                return match.group(0)
        
        # 使用正则表达式替换
        pattern = r'data\[(\d+)\]'
        updated_content = re.sub(pattern, replace_data_index, content)
        
        # 如果内容有变化，写回文件
        if updated_content != content:
            # 创建备份文件
            backup_path = file_path + '.backup'
            with open(backup_path, 'w', encoding='utf-8') as f:
                f.write(content)
            print(f"已创建备份文件: {backup_path}")
            
            # 写回更新后的内容
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(updated_content)
            print(f"已更新文件: {file_path}")
            return True
        else:
            print(f"文件无需更新: {file_path}")
            return False
            
    except Exception as e:
        print(f"处理文件 {file_path} 时出错: {e}")
        return False

def main():
    """
    主函数
    """
    # 指定要处理的文件
    target_file = "/home/hang/deploy/cdroid_ros2_ws/src/cdroid_ros2_deploy/plot_juggler/hihonr_leg_velocity2.xml"
    
    print("开始更新XML文件中的data数组索引...")
    print("将所有data[*]中的索引减去3")
    print("-" * 50)
    
    if not os.path.exists(target_file):
        print(f"文件不存在: {target_file}")
        return
    
    print(f"处理文件: {target_file}")
    
    # 更新文件
    if update_data_indices_in_file(target_file):
        print("更新成功！")
    else:
        print("更新失败或文件无需更新")
    
    print("-" * 50)
    print("处理完成！")

if __name__ == "__main__":
    main() 