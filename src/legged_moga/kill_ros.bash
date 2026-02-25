#!/bin/bash
set -e

echo -e "\n=============================="
echo "🧹 开始安全清理 ROS2 相关进程..."
echo "=============================="

# 先展示一下将要处理的目标
echo -e "\n🔍 将匹配的进程："
pgrep -alf -x "ros2|rviz2|gzserver|gzclient|gazebo|ign|ign-gazebo|isaac-sim" 2>/dev/null || echo "（暂无）"

# 逐类精确结束（不使用 -f 的宽匹配）
for name in ros2 rviz2 gzserver gzclient gazebo ign ign-gazebo isaac-sim; do
  if pgrep -x "$name" >/dev/null; then
    pkill -9 -x "$name" && echo "✅ 已结束：$name"
  else
    echo "⚪ 无进程：$name"
  fi
done

# 有些 launch/run 由 ros2 子进程挂着，再额外收一遍仅匹配“ros2 launch/run”
pgrep -a ros2 | grep -E "\bros2\s+(launch|run)\b" | awk '{print $1}' | xargs -r kill -9 && echo "✅ 已结束：ros2 launch/run 子进程"

# DDS（仅在确实存在同名独立进程时才杀）
for name in fastdds rtidds cyclonedds; do
  if pgrep -x "$name" >/dev/null; then
    pkill -9 -x "$name" && echo "✅ 已结束：$name"
  fi
done

# 清理残留的临时文件（只清理 ros 相关前缀）
echo -e "\n🧽 清理临时文件..."
rm -rf /tmp/ros2* /tmp/launch_* /dev/shm/ros2* 2>/dev/null || true
echo "✅ /tmp 与 /dev/shm 清理完成。"

# 可选：停掉 ROS2 守护进程，防止下一次发现旧缓存
command -v ros2 >/dev/null && ros2 daemon stop >/dev/null 2>&1 && echo "✅ 已停止 ros2 daemon"

echo -e "\n=============================="
echo "✅ ROS2 清理完成（输入法/VSCode 安全）。"
echo "=============================="
