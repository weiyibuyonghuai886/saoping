#!/usr/bin/env python3
"""
Launch file for domain_bridge
Bridges /joint_states from Domain 1 to Domain 0 as /remote/joint_states
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Domain Bridge 配置文件
    bridge_config = PathJoinSubstitution(
        [
            FindPackageShare("hardware_sim_bridge_test"),
            "config",
            "domain_bridge.yaml",
        ]
    )

    # Domain Bridge Node (在 Domain 1 运行，桥接到 Domain 0)
    # domain_bridge 使用位置参数而不是ROS参数
    domain_bridge_node = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name="domain_bridge",
        arguments=[
            bridge_config,
            "--from", "66",  # 从 domain 1
            "--to", "77",    # 到 domain 0
        ],
        output="both",
    )

    return LaunchDescription([
        domain_bridge_node,
    ])

