import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import Shutdown


def generate_launch_description():

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    cmd_vel = LaunchConfiguration("cmd_vel", default='input_joy/cmd_vel')
    parameters_file = PathJoinSubstitution(
        [
            FindPackageShare("rl_controllers"),
            "config",
            "joy.yaml",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("legged_moga_description"), "urdf", "moga.xacro"]
            ),
            " ",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rl_controllers"),
            "config",
            "moga_real.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        sigterm_timeout='5',  # SIGTERM 后等待5秒
        sigkill_timeout='2',  # 然后 SIGKILL 等待2秒
        on_exit=Shutdown(),   # 如果控制节点退出，关闭整个 launch
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rl_controllers", "--controller-manager", "controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Launch rviz visualization
    pkg_dir = get_package_share_directory('legged_moga_description')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', [os.path.join(pkg_dir, 'rviz', 'moga.rviz')]]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'autorepeat_rate': 20.0}],
        sigterm_timeout='2',
        sigkill_timeout='1',
        # remappings=[('/joy', 'input_joy')]
    )

    teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        parameters=[parameters_file],
        sigterm_timeout='2',
        sigkill_timeout='1',
    )

    incrementer_node = Node(
        package='joy_teleop',
        executable='incrementer_server',
        name='incrementer',
        namespace='torso_controller',
        sigterm_timeout='2',
        sigkill_timeout='1',
    )

    yesense_imu_node = Node(
            package="yesense_std_ros2",
            executable="yesense_node_publisher",
            name="yesense_pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"serial_port": "/dev/ttyACM0"},
                {"baud_rate": 460800},
                {"frame_id": "base_imu"},
                {"driver_type": "ros_serial"}
            ],
            sigterm_timeout='2',
            sigkill_timeout='1',
        )

    nodes = [
        DeclareLaunchArgument('cmd_vel', default_value='input_joy/cmd_vel'),
        DeclareLaunchArgument('teleop_config', default_value=parameters_file),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        rviz2_node,
        joy_node,
        # 暂时注释掉有 shutdown 问题的节点
        # teleop_node,
        # incrementer_node,
        yesense_imu_node,
    ]

    return LaunchDescription(nodes)
