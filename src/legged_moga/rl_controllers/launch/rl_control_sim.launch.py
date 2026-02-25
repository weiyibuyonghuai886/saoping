from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,RegisterEventHandler,OpaqueFunction,ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,PythonExpression,Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def launch_setup_gazebo(context, *args, **kwargs):
    # Launch configuration variables specific to simulation
    # 定义两个启动配置变量
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world_file = LaunchConfiguration('world_file')
    
    # Constants for paths to different files and folders
    # 定义包的名称，和gazebo参数文件路径
    package_name = 'legged_moga_description'
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    # Set the path to different files and folders.  
    pkg_gazebo_ros = os.path.join(get_package_share_directory("gazebo_ros"))

    # 启动gazebo
    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    )

    return [
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
    ]


def launch_setup_spawn(context, *args, **kwargs):
        # Get the launch configuration
    package_name='moga' #<--- CHANGE ME
    # 从启动配置中获取参数
    spawn_x_val = LaunchConfiguration('spawn_x_val').perform(context)
    spawn_y_val = LaunchConfiguration('spawn_y_val').perform(context)
    spawn_yaw_val = LaunchConfiguration('spawn_yaw_val').perform(context)
    spawn_yaw_val = LaunchConfiguration('spawn_yaw_val').perform(context)
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # xacro路径
    pkg_share = os.path.join(get_package_share_directory("legged_moga_description"))
    default_model_path = os.path.join(pkg_share, "urdf/moga_sim.xacro")
    

    # 创建机器人状态发布节点:
    robot_description_config = ParameterValue(Command(['xacro ', default_model_path]), value_type=str)
    params = {'robot_description': robot_description_config, 'use_sim_time': True}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 创建 Gazebo 实体生成节点
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', package_name,
                                    '-x', spawn_x_val,
                                    '-y', spawn_y_val,
                                    '-z', "1.5",
                                    '-Y', spawn_yaw_val],
                                     output='screen')
    

    # 加载关节状态控制器:
    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen',
            # parameters=[rl_param_dir]

    )



    # 延迟加载
    delayed_joint_broad_spawner =   RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )   

    # 定义需要加载的所有控制器的名称列表。
    controller_names = [
        "rl_controllers"
    ]

    # Create a list to hold the event handlers for the controllers
    controller_event_handlers = []

    paramss = {'robot':True}
    # 加载控制器
    # Create the load and event handler actions for each controller
    for controller_name in controller_names:
        load_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_name],
            output='screen',
        )
        # controller_spawners.append(load_controller)
        
        # Create an event handler to load each controller after the previous one has finished
        delayed_diff_drive_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_controller],
            )
        )
        controller_event_handlers.append(delayed_diff_drive_spawner)

    # Add all nodes and event handlers to the launch description
    ld = [
        # control_node,
        node_robot_state_publisher,
        spawn_entity,
        delayed_joint_broad_spawner,
    ] + controller_event_handlers


    return ld

pkg_dir = get_package_share_directory('legged_moga_description')
rviz2_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', [os.path.join(pkg_dir, 'rviz', 'urdf.rviz')]]
)
def generate_launch_description():
    # 获取世界文件路径
    training_world_path = os.path.join(
        get_package_share_directory('legged_moga_description'),
        'worlds',
        'training_world.world'
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to execute gzclient')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')
    
    # 添加世界文件参数
    declare_world_file_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value=training_world_path,
        description='Path to the Gazebo world file')
    
    declare_spawn_x_val = DeclareLaunchArgument('spawn_x_val', default_value='0.0', description='Spawn X Value')
    declare_spawn_y_val = DeclareLaunchArgument('spawn_y_val', default_value='0.0', description='Spawn Y Value')
    declare_spawn_yaw_val = DeclareLaunchArgument('spawn_yaw_val', default_value='0.00', description='Spawn Yaw Value')
    declare_use_sim_time =    DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true')
    
        
    parameters_file = os.path.join(
        get_package_share_directory('rl_controllers'),
        'config', 'joy.yaml'
    )

    ld = LaunchDescription([
        launch.actions.DeclareLaunchArgument('cmd_vel', default_value='input_joy/cmd_vel'),
        launch.actions.DeclareLaunchArgument('teleop_config', default_value=parameters_file),

        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'autorepeat_rate': 20.0}],
            # remappings=[('/joy', 'input_joy')]
            )
    ])

    ld.add_action(launch_ros.actions.Node(
            package='joy_teleop', executable='joy_teleop',
            parameters=[launch.substitutions.LaunchConfiguration('teleop_config')]))

    ld.add_action(launch_ros.actions.Node(
            package='joy_teleop', executable='incrementer_server',
            name='incrementer', namespace='torso_controller'))
    
    return LaunchDescription([
        declare_use_simulator_cmd,
        declare_simulator_cmd,
        declare_world_file_cmd,
        declare_spawn_x_val,
        declare_spawn_y_val,
        declare_spawn_yaw_val,
        declare_use_sim_time,
        rviz2_node,
        OpaqueFunction(function=launch_setup_gazebo),
        OpaqueFunction(function=launch_setup_spawn),
        ld
    ])
    
    
