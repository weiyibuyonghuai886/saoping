/*
 * NOTE:
 * 本代码只包含所有的除 ros2_control 框架外的自定义函数
 */

#include "LeggedSystem.h"
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <ifaddrs.h>
#include <net/if.h>
#include <cstring>
#include <iostream>
#include <ostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "utilities.h"

namespace legged {

// 初始化踝关节解算器
void LeggedSystemHardware::initializeAnkleSolvers() {
    // 获取机器人配置 - 参考honor_example.cpp中的配置
    std::string motor_config = "X"; // 电机朝向配置 (X or x: X axis, Y or y: Y axis)
    std::string ankle_config = "RTP"; // 踝关节配置 (RTP or rtp: 脚踝先pitch后roll, PTR or ptr: 先roll后pitch)

    // NOTE: 配置左脚踝关节解算器参数
    Eigen::Vector3d l_A1_0 = {0.034341889487, 0.000006600510, 0.215911000000};
    Eigen::Vector3d l_A2_0 = {0.034398989487, -0.000012962400, 0.150911000000};
    Eigen::Vector3d l_B1_0 = {0.034349689488, 0.014506700000, 0.207037000000};
    Eigen::Vector3d l_B2_0 = {0.034406789487, -0.014463700000, 0.141957000000};
    Eigen::Vector3d l_C1_0 = {0.022227589488, 0.016505100000, -0.008504140000};
    Eigen::Vector3d l_C2_0 = {0.022227589488, -0.016494900000, -0.008478260000};
    double d_ankle_left = 0.015; // 脚踝 roll 到 pitch 轴的直线距离 (m)
    Eigen::Vector2d roll_range_left = {-25 * DEG_TO_RAD, 25 * DEG_TO_RAD};
    Eigen::Vector2d pitch_range_left = {-75 * DEG_TO_RAD, 8 * DEG_TO_RAD};
    Eigen::Vector2d left_motor_range_left = {-180 * DEG_TO_RAD, 180 * DEG_TO_RAD};
    Eigen::Vector2d right_motor_range_left = {-180 * DEG_TO_RAD, 180 * DEG_TO_RAD};

    // NOTE: 配置右脚踝关节解算器参数
    Eigen::Vector3d r_A1_0 = {0.034398989487, 0.000012962400, 0.150911000000};
    Eigen::Vector3d r_A2_0 = {0.034341889487, -0.000006600510, 0.215911000000};
    Eigen::Vector3d r_B1_0 = {0.034406789487, 0.014463700000, 0.141957000000};
    Eigen::Vector3d r_B2_0 = {0.034349689488, -0.014506700000, 0.207037000000};
    Eigen::Vector3d r_C1_0 = {0.022227589488, 0.016494900000, -0.008478260000};
    Eigen::Vector3d r_C2_0 = {0.022227589488, -0.016505100000, -0.008504140000};
    double d_ankle_right = 0.015; // 脚踝 roll 到 pitch 轴的直线距离 (m)
    Eigen::Vector2d roll_range_right = {-25 * DEG_TO_RAD, 25 * DEG_TO_RAD};
    Eigen::Vector2d pitch_range_right = {-75 * DEG_TO_RAD, 8 * DEG_TO_RAD};
    Eigen::Vector2d left_motor_range_right = {-180 * DEG_TO_RAD, 180 * DEG_TO_RAD};
    Eigen::Vector2d right_motor_range_right = {-180 * DEG_TO_RAD, 180 * DEG_TO_RAD};

    left_ankle_left_motor_pos_offset_ = +0.499190;          // 左脚踝左电机位置偏置
    left_ankle_right_motor_pos_offset_ = -0.572052;          // 左脚踝右电机位置偏置
    right_ankle_left_motor_pos_offset_ = +0.581590;         // 右脚踝左电机位置偏置
    right_ankle_right_motor_pos_offset_ = -0.507964;         // 右脚踝右电机位置偏置
    ankle_pitch_pos_offset_ = 12.3920 * DEG_TO_RAD;        // 腿部倾斜角度(12.7°转弧度)

    // 初始化左脚踝关节解算器
    left_ankle_solver_ = std::make_unique<HumanoidParallelAnkleSolver>(
        motor_config, ankle_config,
        l_A1_0, l_A2_0,
        l_B1_0, l_B2_0,
        l_C1_0, l_C2_0,
        d_ankle_left,
        roll_range_left, pitch_range_left,
        left_motor_range_left, right_motor_range_left
    );

    // 初始化右脚踝关节解算器（使用相同的参数）
    right_ankle_solver_ = std::make_unique<HumanoidParallelAnkleSolver>(
        motor_config, ankle_config,
        r_A1_0, r_A2_0,
        r_B1_0, r_B2_0,
        r_C1_0, r_C2_0,
        d_ankle_right,
        roll_range_right, pitch_range_right,
        right_motor_range_right, left_motor_range_right
    );

    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Ankle solvers initialized successfully");
}

// 加载关节名称参数
void LeggedSystemHardware::loadJointNameParams() {
    try {
        // 获取包路径
        std::string package_path = ament_index_cpp::get_package_share_directory("legged_system");
        std::string config_file = package_path + "/config/joint_name_params.yaml";

        // 检查配置文件是否存在
        if (!std::filesystem::exists(config_file)) {
            RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                        "Joint name params config file not found: %s", config_file.c_str());
            throw std::runtime_error("Joint name params config file not found");
        }

        // 加载YAML文件
        YAML::Node config = YAML::LoadFile(config_file);

        // 清空现有数据
        all_joint_names_.clear();

        // 读取关节名称参数
        if (config["joint_name_parameters"]) {
            for (const auto& joint_config : config["joint_name_parameters"]) {
                if (joint_config.IsSequence() && joint_config.size() >= 8) {
                    JointNameParamsStru joint_name_param;
                    joint_name_param.name = joint_config[0].as<std::string>();
                    joint_name_param.directionMotor_ = joint_config[1].as<int>();
                    joint_name_param.biasMotor_ = joint_config[2].as<double>();
                    joint_name_param.pos_limit_positive_ = joint_config[3].as<double>();
                    joint_name_param.pos_limit_negative_ = joint_config[4].as<double>();
                    joint_name_param.vel_limit_ = joint_config[5].as<double>();
                    joint_name_param.tau_limit_ = joint_config[6].as<double>();
                    joint_name_param.ignore_limit_check_ = joint_config[7].as<bool>();
                    all_joint_names_.push_back(joint_name_param);
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("LeggedSystemHardware"),
                               "Invalid joint name parameter format in config file (expected 8 values)");
                }
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                        "No 'joint_name_parameters' section found in config file");
            throw std::runtime_error("No joint_name_parameters section found");
        }

        // 验证加载的参数数量
        if (all_joint_names_.size() != NUM_MOTOR) {
            RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                        "Expected %d joint name parameters, but got %zu",
                        NUM_MOTOR, all_joint_names_.size());
            throw std::runtime_error("Joint name parameter count mismatch");
        }

        RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"),
                   "Successfully loaded %zu joint name parameters from %s",
                   all_joint_names_.size(), config_file.c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "Failed to load joint name parameters: %s", e.what());
        throw;
    }
}

// 初始化关节参数数组
void LeggedSystemHardware::initializeJointParams() {
    try {
        // 获取包路径
        std::string package_path = ament_index_cpp::get_package_share_directory("legged_system");
        std::string config_file = package_path + "/config/joint_params.yaml";

        RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"),
                   "Loading joint parameters from: %s", config_file.c_str());

        // 加载YAML配置文件
        YAML::Node config = YAML::LoadFile(config_file);

        if (!config["joint_parameters"]) {
            RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                        "No 'joint_parameters' section found in config file");
            return;
        }

        YAML::Node joint_params = config["joint_parameters"];
        int joint_count = 0;

        // 遍历配置文件中的关节参数（数组格式）
        for (const auto& joint_item : joint_params) {
            if (!joint_item.IsSequence() || joint_item.size() < 9) {
                RCLCPP_WARN(rclcpp::get_logger("LeggedSystemHardware"), "Invalid joint parameter format, skipping");
                continue;
            }
            std::string joint_name = joint_item[0].as<std::string>();
            std::string motor_index = joint_item[1].as<std::string>();
            int joint_index = joint_item[2].as<int>();
            int direction_motor = joint_item[3].as<int>();
            float pos_limit_positive = joint_item[4].as<float>();
            float pos_limit_negative = joint_item[5].as<float>();
            float vel_limit = joint_item[6].as<float>();
            float tau_limit = joint_item[7].as<float>();
            float bias_motor = joint_item[8].as<float>();
            bool is_ankle_motor = joint_item[9].as<bool>();
            int ankle_solver_motor_index = -1;
            float motor_pos_limit_positive = 0.0;
            float motor_pos_limit_negative = 0.0;
            float motor_vel_limit = 0.0;
            float motor_tau_limit = 0.0;

            if (is_ankle_motor) {
                ankle_solver_motor_index = joint_item[10].as<int>();
                motor_pos_limit_positive = joint_item[11].as<float>();
                motor_pos_limit_negative = joint_item[12].as<float>();
                motor_vel_limit = joint_item[13].as<float>();
                motor_tau_limit = joint_item[14].as<float>();
            }

            // 查找对应的关节索引
            int urdf_joint_index = -1;
            for (size_t i = 0; i < info_.joints.size(); i++) {
                if (info_.joints[i].name == joint_name) {
                    urdf_joint_index = i;
                    break;
                }
            }
            if (urdf_joint_index == -1) {
                RCLCPP_WARN(rclcpp::get_logger("LeggedSystemHardware"), "Joint '%s' not found in URDF, skipping", joint_name.c_str());
                continue;
            }

            jointParams_[urdf_joint_index].jointName = joint_name.c_str();
            jointParams_[urdf_joint_index].motorIndex = motor_index;
            jointParams_[urdf_joint_index].jointIndex = joint_index;
            jointParams_[urdf_joint_index].directionMotor_ = direction_motor;
            jointParams_[urdf_joint_index].pos_limit_positive_ = pos_limit_positive;
            jointParams_[urdf_joint_index].pos_limit_negative_ = pos_limit_negative;
            jointParams_[urdf_joint_index].vel_limit_ = vel_limit;
            jointParams_[urdf_joint_index].tau_limit_ = tau_limit;
            jointParams_[urdf_joint_index].biasMotor_ = bias_motor;
            jointParams_[urdf_joint_index].isAnkleMotor_ = is_ankle_motor;
            jointParams_[urdf_joint_index].ankleSolverMotorIndex_ = ankle_solver_motor_index;
            jointParams_[urdf_joint_index].motor_pos_limit_positive_ = motor_pos_limit_positive;
            jointParams_[urdf_joint_index].motor_pos_limit_negative_ = motor_pos_limit_negative;
            jointParams_[urdf_joint_index].motor_vel_limit_ = motor_vel_limit;
            jointParams_[urdf_joint_index].motor_tau_limit_ = motor_tau_limit;

            joint_count++;

            RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"),
                        "Loaded joint %s: motor_id=%s, joint_index=%d, urdf_joint_index=%d, direction=%d, biasMotor_=%.4f",
                        joint_name.c_str(),
                        motor_index.c_str(),
                        joint_index,
                        urdf_joint_index,
                        direction_motor,
                        bias_motor);
        }

        RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"),
                   "Successfully loaded parameters for %d joints", joint_count);

        // 构建motorIndex到jointParams索引的哈希表
        motorIndexToJointParamsIndex_.clear();
        for (int i = 0; i < NUM_MOTOR; i++) {
            if (!jointParams_[i].motorIndex.empty()) {  // 检查motorIndex是否为空字符串
                motorIndexToJointParamsIndex_[jointParams_[i].motorIndex] = i;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"),
                   "Motor index lookup table built with %zu entries", motorIndexToJointParamsIndex_.size());

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "Error parsing YAML config file: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "Error loading joint parameters: %s", e.what());
    }
}

// 初始化关节速度滤波
void LeggedSystemHardware::initializeJointVelFilters() {
    // 初始化移动平均滤波器（样本数量）
    leg_l1_joint_vel_filter_ = std::make_unique<MovingAverageFilter>(10);
    leg_l2_joint_vel_filter_ = std::make_unique<MovingAverageFilter>(10);
    leg_l3_joint_vel_filter_ = std::make_unique<MovingAverageFilter>(10);
    leg_l4_joint_vel_filter_ = std::make_unique<MovingAverageFilter>(10);

    leg_r1_joint_vel_filter_ = std::make_unique<MovingAverageFilter>(10);
    leg_r2_joint_vel_filter_ = std::make_unique<MovingAverageFilter>(10);
    leg_r3_joint_vel_filter_ = std::make_unique<MovingAverageFilter>(10);
    leg_r4_joint_vel_filter_ = std::make_unique<MovingAverageFilter>(10);

    // 初始化脚踝电机速度滤波器
    left_ankle_left_motor_vel_filter_ = std::make_unique<MovingAverageFilter>(1);
    left_ankle_right_motor_vel_filter_ = std::make_unique<MovingAverageFilter>(1);
    right_ankle_left_motor_vel_filter_ = std::make_unique<MovingAverageFilter>(1);
    right_ankle_right_motor_vel_filter_ = std::make_unique<MovingAverageFilter>(1);

    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Joint velocity filters initialized successfully");
}

// 初始化ROS踝关节解算消息
void LeggedSystemHardware::initializeAnkleDebugMessages() {
    // 初始化ROS踝关节解算消息
    real_input_motor_pos_msg_.data.assign(4, 0.0);
    real_input_motor_vel_msg_.data.assign(4, 0.0);
    real_input_motor_tau_msg_.data.assign(4, 0.0);
    solver_mtj_input_motor_pos_msg_.data.assign(4, 0.0);
    solver_mtj_input_motor_vel_msg_.data.assign(4, 0.0);
    solver_mtj_input_motor_tau_msg_.data.assign(4, 0.0);
    solver_mtj_output_ankle_pos_msg_.data.assign(4, 0.0);
    solver_mtj_output_ankle_vel_msg_.data.assign(4, 0.0);
    solver_mtj_output_ankle_tau_msg_.data.assign(4, 0.0);
    solver_jtm_output_motor_pos_msg_.data.assign(4, 0.0);
    solver_jtm_output_motor_vel_msg_.data.assign(4, 0.0);
    solver_jtm_output_motor_tau_msg_.data.assign(4, 0.0);
    real_output_motor_pos_msg_.data.assign(4, 0.0);
    real_output_motor_vel_msg_.data.assign(4, 0.0);
    real_output_motor_tau_msg_.data.assign(4, 0.0);
    read_time_msg_.data = 0.0;
    write_time_msg_.data = 0.0;
    read_period_msg_.data = 0.0;
    write_period_msg_.data = 0.0;
    cal_read_time_msg_.data = 0.0;
    cal_write_time_msg_.data = 0.0;

    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Ankle debug messages initialized successfully");
}

// 初始化脚踝解算器调试发布者
void LeggedSystemHardware::initializeAnkleDebugPublishers() {
    // Read
    real_input_motor_pos_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/real_input_motor_pos", 1);   // 实际电机位置
    real_input_motor_vel_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/real_input_motor_vel", 1);
    real_input_motor_tau_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/real_input_motor_tau", 1);
    solver_mtj_input_motor_pos_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/solver_mtj_input_motor_pos", 1);  // 内部电机位置
    solver_mtj_input_motor_vel_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/solver_mtj_input_motor_vel", 1);
    solver_mtj_input_motor_tau_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/solver_mtj_input_motor_tau", 1);
    solver_mtj_output_ankle_pos_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/solver_mtj_output_ankle_pos", 1);  // 解算脚踝位置
    solver_mtj_output_ankle_vel_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/solver_mtj_output_ankle_vel", 1);
    solver_mtj_output_ankle_tau_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/solver_mtj_output_ankle_tau", 1);

    // Write
    solver_jtm_output_motor_pos_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/solver_jtm_output_motor_pos", 1);  // 解算脚踝位置
    solver_jtm_output_motor_vel_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/solver_jtm_output_motor_vel", 1);
    solver_jtm_output_motor_tau_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/solver_jtm_output_motor_tau", 1);
    real_output_motor_pos_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/real_output_motor_pos", 1);  // 解算电机位置
    real_output_motor_vel_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/real_output_motor_vel", 1);
    real_output_motor_tau_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/data_analysis/real_output_motor_tau", 1);
    read_time_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/data_analysis/read_time", 1);
    write_time_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/data_analysis/write_time", 1);
    read_period_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/data_analysis/read_period", 1);
    write_period_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/data_analysis/write_period", 1);
    cal_read_time_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/data_analysis/cal_read_time", 1);
    cal_write_time_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/data_analysis/cal_write_time", 1);

    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Ankle debug publishers initialized successfully");
}

// IMU话题回调函数
void LeggedSystemHardware::callbackImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    latest_imu_ = *msg;

    // 打印接收到的 IMU 数据（每隔一段时间打印一次）
    // static int imu_msg_count = 0;
    // imu_msg_count++;
    // if (imu_msg_count % 1000 == 0) {  // 每50条消息打印一次
    //     RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"),
    //         "✓ 收到 IMU 消息 #%d - 姿态: [x:%.3f, y:%.3f, z:%.3f, w:%.3f], 角速度: [%.3f, %.3f, %.3f], 加速度: [%.3f, %.3f, %.3f]",
    //         imu_msg_count,
    //         msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
    //         msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
    //         msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    // }
}

void LeggedSystemHardware::callbackJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_state_ = *msg;

    // 打印接收到的 Joint State 数据（每隔一段时间打印一次）
    // static int joint_state_msg_count = 0;
    // joint_state_msg_count++;
    // if (joint_state_msg_count % 1000 == 0) {  // 每100条消息打印一次
    //     // 打印前6个关节的数据作为示例（避免输出太长）
    //     int num_joints_to_print = std::min(6, (int)msg->name.size());
    //     std::stringstream ss;
    //     ss << "✓ 收到 Joint State 消息 #" << joint_state_msg_count
    //        << " - 总共 " << msg->name.size() << " 个关节";

    //     if (num_joints_to_print > 0) {
    //         ss << " | 前" << num_joints_to_print << "个关节: ";
    //         for (int i = 0; i < num_joints_to_print; i++) {
    //             ss << msg->name[i] << "=";
    //             if (i < (int)msg->position.size()) {
    //                 ss << std::fixed << std::setprecision(3) << msg->position[i];
    //             } else {
    //                 ss << "N/A";
    //             }
    //             if (i < num_joints_to_print - 1) ss << ", ";
    //         }
    //     }

        // RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "%s", ss.str().c_str());
    // }
}

// 重置关节限位状态回调函数
void LeggedSystemHardware::callbackWriteJointLimitStatusReset(const std_msgs::msg::Float32::SharedPtr msg) {
    (void)msg;  // 消除未使用参数警告
    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Received reset joint limit exceeded status command from joystick");
    writeJointLimitStatusReset();
}

// 设置硬件通信
void LeggedSystemHardware::activateHardwareCommunication() {

    // imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    //     "/xlab/hr/body_upper_imu", 10, std::bind(&LeggedSystemHardware::callbackImu, this, std::placeholders::_1));
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data_ros", 10, std::bind(&LeggedSystemHardware::callbackImu, this, std::placeholders::_1));
    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "IMU subscription created");

    // 初始化自定义控制命令发布者
    custom_cmd_pub_ = this->node_->create_publisher<custom_controller_msgs::msg::CustomControlCommand>("/robot_controller/commands", 10);
    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Custom control command publisher created");

    // 初始化关节状态反馈发布者
    // joint_state_pub_ = this->node_->create_publisher<sensor_msgs::msg::JointState>("/robot_controller/joint_states", 10);
    // RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Joint state publisher created");

    // 初始化关节状态订阅者（用于接收硬件反馈）
    // 设置 QoS 为 BEST_EFFORT 以匹配发布者
    joint_state_sub_ = this->node_->create_subscription<sensor_msgs::msg::JointState>(
        "/moga_joint_state_broadcaster/joint_states",  // 订阅硬件反馈的话题
        rclcpp::QoS(10).best_effort(),
        std::bind(&LeggedSystemHardware::callbackJointState, this, std::placeholders::_1)
    );
    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Joint state subscriber created for /moga_joint_state_broadcaster/joint_states");

    // 创建重置关节限位状态订阅器
    reset_joint_limit_status_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/reset_joint_limit_status", 10, std::bind(&LeggedSystemHardware::callbackWriteJointLimitStatusReset, this, std::placeholders::_1));
    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Reset joint limit subscription created");

    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Hardware communication setup completed");
}

void LeggedSystemHardware::readMotorData(MotorDataStru joint_data[NUM_MOTOR],
                                        AnkleDataStru &left_ankle_motor_data,
                                        AnkleDataStru &right_ankle_motor_data,
                                        bool &left_ankle_data_ready,
                                        bool &right_ankle_data_ready) {
    // 从JointState消息中提取关节状态数据并应用电机方向和关节偏置
    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    // 检查消息是否有效
    if (latest_joint_state_.name.empty()) {
        return;  // 尚未接收到任何数据
    }

    // 遍历接收到的所有关节
    for (size_t i = 0; i < latest_joint_state_.name.size(); i++) {
        const std::string& joint_name = latest_joint_state_.name[i];

        // 使用哈希表快速查找对应的关节参数索引
        auto it = motorIndexToJointParamsIndex_.find(joint_name);
        if (it == motorIndexToJointParamsIndex_.end()) {
            continue; // 如果找不到对应的关节，跳过
        }

        int param_index = it->second;
        const auto& jointParam = jointParams_[param_index];
        // std::cerr << "joint_name: " << joint_name << " jointParam.jointName: " << jointParam.jointName << " param_index: " << param_index << std::endl;

        // 提取位置、速度、力矩数据（如果可用）
        double pos = (i < latest_joint_state_.position.size()) ? latest_joint_state_.position[i] : 0.0;
        double vel = (i < latest_joint_state_.velocity.size()) ? latest_joint_state_.velocity[i] : 0.0;
        double tau = (i < latest_joint_state_.effort.size()) ? latest_joint_state_.effort[i] : 0.0;

        // 应用电机方向和关节偏置
        joint_data[jointParam.jointIndex].pos_ = pos * jointParam.directionMotor_ + jointParam.biasMotor_;
        joint_data[jointParam.jointIndex].vel_ = vel * jointParam.directionMotor_;
        joint_data[jointParam.jointIndex].tau_ = tau * jointParam.directionMotor_;

        // 对于脚踝关节，收集电机数据
        if (joint_name == "leg_joint_5") {  // Left Ankle Pitch
            left_ankle_motor_data.pos_(0) = pos * jointParam.directionMotor_ + jointParam.biasMotor_;
            left_ankle_motor_data.vel_(0) = vel * jointParam.directionMotor_;
            left_ankle_motor_data.tau_(0) = tau * jointParam.directionMotor_;
        } else if (joint_name == "leg_joint_6") {  // Left Ankle Roll
            left_ankle_motor_data.pos_(1) = pos * jointParam.directionMotor_ + jointParam.biasMotor_;
            left_ankle_motor_data.vel_(1) = vel * jointParam.directionMotor_;
            left_ankle_motor_data.tau_(1) = tau * jointParam.directionMotor_;
            left_ankle_data_ready = true;
        } else if (joint_name == "leg_joint_11") {  // Right Ankle Pitch
            right_ankle_motor_data.pos_(1) = pos * jointParam.directionMotor_ + jointParam.biasMotor_;
            right_ankle_motor_data.vel_(1) = vel * jointParam.directionMotor_;
            right_ankle_motor_data.tau_(1) = tau * jointParam.directionMotor_;
        } else if (joint_name == "leg_joint_12") {  // Right Ankle Roll
            right_ankle_motor_data.pos_(0) = pos * jointParam.directionMotor_ + jointParam.biasMotor_;
            right_ankle_motor_data.vel_(0) = vel * jointParam.directionMotor_;
            right_ankle_motor_data.tau_(0) = tau * jointParam.directionMotor_;
            right_ankle_data_ready = true;
        }
    }
}

void LeggedSystemHardware::readApplyFilters(MotorDataStru joint_data[NUM_MOTOR],
                                            AnkleDataStru &left_ankle_motor_data,
                                            AnkleDataStru &right_ankle_motor_data) {
    // 对关节位置数据应用移动平均滤波
    joint_data[0].vel_ = applyMovingAverageFilter(joint_data[0].vel_, *leg_l1_joint_vel_filter_);
    joint_data[1].vel_ = applyMovingAverageFilter(joint_data[1].vel_, *leg_l2_joint_vel_filter_);
    joint_data[2].vel_ = applyMovingAverageFilter(joint_data[2].vel_, *leg_l3_joint_vel_filter_);
    joint_data[3].vel_ = applyMovingAverageFilter(joint_data[3].vel_, *leg_l4_joint_vel_filter_);

    joint_data[6].vel_ = applyMovingAverageFilter(joint_data[6].vel_, *leg_r1_joint_vel_filter_);
    joint_data[7].vel_ = applyMovingAverageFilter(joint_data[7].vel_, *leg_r2_joint_vel_filter_);
    joint_data[8].vel_ = applyMovingAverageFilter(joint_data[8].vel_, *leg_r3_joint_vel_filter_);
    joint_data[9].vel_ = applyMovingAverageFilter(joint_data[9].vel_, *leg_r4_joint_vel_filter_);

    // 对脚踝电机速度应用移动平均滤波
    left_ankle_motor_data.vel_(0) = applyMovingAverageFilter(left_ankle_motor_data.vel_(0), *left_ankle_left_motor_vel_filter_);
    left_ankle_motor_data.vel_(1) = applyMovingAverageFilter(left_ankle_motor_data.vel_(1), *left_ankle_right_motor_vel_filter_);
    right_ankle_motor_data.vel_(0) = applyMovingAverageFilter(right_ankle_motor_data.vel_(0), *right_ankle_left_motor_vel_filter_);
    right_ankle_motor_data.vel_(1) = applyMovingAverageFilter(right_ankle_motor_data.vel_(1), *right_ankle_right_motor_vel_filter_);
}

AnkleDataStru LeggedSystemHardware::readCalculateAnkleRollPitchFromMotor(std::string ankle_name, AnkleDataStru &ankle_motor_data) {

#if ANKLE_SOLVER_DEBUG_MODE_ENABLE
    // 获取真实左脚踝关节电机数据
    if (ankle_name == "left") {
        real_input_motor_pos_msg_.data[0] = ankle_motor_data.pos_(0) * RAD_TO_DEG;
        real_input_motor_pos_msg_.data[1] = ankle_motor_data.pos_(1) * RAD_TO_DEG;
        real_input_motor_vel_msg_.data[0] = ankle_motor_data.vel_(0) * RAD_TO_DEG;
        real_input_motor_vel_msg_.data[1] = ankle_motor_data.vel_(1) * RAD_TO_DEG;
        real_input_motor_tau_msg_.data[0] = ankle_motor_data.tau_(0);
        real_input_motor_tau_msg_.data[1] = ankle_motor_data.tau_(1);
    } else if (ankle_name == "right") {
        real_input_motor_pos_msg_.data[2] = ankle_motor_data.pos_(0) * RAD_TO_DEG;
        real_input_motor_pos_msg_.data[3] = ankle_motor_data.pos_(1) * RAD_TO_DEG;
        real_input_motor_vel_msg_.data[2] = ankle_motor_data.vel_(0) * RAD_TO_DEG;
        real_input_motor_vel_msg_.data[3] = ankle_motor_data.vel_(1) * RAD_TO_DEG;
        real_input_motor_tau_msg_.data[2] = ankle_motor_data.tau_(0);
        real_input_motor_tau_msg_.data[3] = ankle_motor_data.tau_(1);
    }
#endif // ANKLE_SOLVER_DEBUG_MODE_ENABLE

    // 将实际电机角度转换为解算器电机角度
    if (ankle_name == "left") {
        ankle_motor_data.pos_(0) += left_ankle_left_motor_pos_offset_;
        ankle_motor_data.pos_(1) += left_ankle_right_motor_pos_offset_;
    } else if (ankle_name == "right") {
        ankle_motor_data.pos_(0) += right_ankle_left_motor_pos_offset_;
        ankle_motor_data.pos_(1) += right_ankle_right_motor_pos_offset_;
    }

#if ANKLE_SOLVER_DEBUG_MODE_ENABLE
    // 获取脚踝解算器输入电机角度数据
    if (ankle_name == "left") {
        solver_mtj_input_motor_pos_msg_.data[0] = ankle_motor_data.pos_(0) * RAD_TO_DEG;
        solver_mtj_input_motor_pos_msg_.data[1] = ankle_motor_data.pos_(1) * RAD_TO_DEG;
        solver_mtj_input_motor_vel_msg_.data[0] = ankle_motor_data.vel_(0) * RAD_TO_DEG;
        solver_mtj_input_motor_vel_msg_.data[1] = ankle_motor_data.vel_(1) * RAD_TO_DEG;
        solver_mtj_input_motor_tau_msg_.data[0] = ankle_motor_data.tau_(0);
        solver_mtj_input_motor_tau_msg_.data[1] = ankle_motor_data.tau_(1);
    } else if (ankle_name == "right") {
        solver_mtj_input_motor_pos_msg_.data[2] = ankle_motor_data.pos_(0) * RAD_TO_DEG;
        solver_mtj_input_motor_pos_msg_.data[3] = ankle_motor_data.pos_(1) * RAD_TO_DEG;
        solver_mtj_input_motor_vel_msg_.data[2] = ankle_motor_data.vel_(0) * RAD_TO_DEG;
        solver_mtj_input_motor_vel_msg_.data[3] = ankle_motor_data.vel_(1) * RAD_TO_DEG;
        solver_mtj_input_motor_tau_msg_.data[2] = ankle_motor_data.tau_(0);
        solver_mtj_input_motor_tau_msg_.data[3] = ankle_motor_data.tau_(1);
    }
#endif // ANKLE_SOLVER_DEBUG_MODE_ENABLE

    // 初始化踝关节脚踝数据(roll, pitch)
    AnkleDataStru ankle_joint_data = AnkleDataStru();

    // 通过电机信息计算踝关节信息
    if (ankle_name == "left") {
        std::tie(ankle_joint_data.pos_, ankle_joint_data.vel_, ankle_joint_data.tau_,
            left_ankle_mtj_num_iterations_, left_ankle_mtj_is_success_, left_ankle_mtj_final_error_)
            = left_ankle_solver_->motorToJoint(ankle_motor_data.pos_, ankle_motor_data.vel_, ankle_motor_data.tau_);
    } else if (ankle_name == "right") {
        std::tie(ankle_joint_data.pos_, ankle_joint_data.vel_, ankle_joint_data.tau_,
            right_ankle_mtj_num_iterations_, right_ankle_mtj_is_success_, right_ankle_mtj_final_error_)
            = right_ankle_solver_->motorToJoint(ankle_motor_data.pos_, ankle_motor_data.vel_, ankle_motor_data.tau_);
    }

#if ANKLE_SOLVER_DEBUG_MODE_ENABLE
    // 获取脚踝解算器输出的脚踝数据
    if (ankle_name == "left") {
        solver_mtj_output_ankle_pos_msg_.data[0] = ankle_joint_data.pos_(0) * RAD_TO_DEG;
        solver_mtj_output_ankle_pos_msg_.data[1] = ankle_joint_data.pos_(1) * RAD_TO_DEG;
        solver_mtj_output_ankle_vel_msg_.data[0] = ankle_joint_data.vel_(0) * RAD_TO_DEG;
        solver_mtj_output_ankle_vel_msg_.data[1] = ankle_joint_data.vel_(1) * RAD_TO_DEG;
        solver_mtj_output_ankle_tau_msg_.data[0] = ankle_joint_data.tau_(0);
        solver_mtj_output_ankle_tau_msg_.data[1] = ankle_joint_data.tau_(1);
    } else if (ankle_name == "right") {
        solver_mtj_output_ankle_pos_msg_.data[2] = ankle_joint_data.pos_(0) * RAD_TO_DEG;
        solver_mtj_output_ankle_pos_msg_.data[3] = ankle_joint_data.pos_(1) * RAD_TO_DEG;
        solver_mtj_output_ankle_vel_msg_.data[2] = ankle_joint_data.vel_(0) * RAD_TO_DEG;
        solver_mtj_output_ankle_vel_msg_.data[3] = ankle_joint_data.vel_(1) * RAD_TO_DEG;
        solver_mtj_output_ankle_tau_msg_.data[2] = ankle_joint_data.tau_(0);
        solver_mtj_output_ankle_tau_msg_.data[3] = ankle_joint_data.tau_(1);
    }
#endif // ANKLE_SOLVER_DEBUG_MODE_ENABLE

    // 将解算器关节角度转换为实际关节角度
    ankle_joint_data.pos_(1) += ankle_pitch_pos_offset_;

    return ankle_joint_data;
}

void LeggedSystemHardware::readUpdateAnkleRollPitchToJointData(MotorDataStru joint_data[NUM_MOTOR],
                                                                AnkleDataStru &left_ankle_joint_data,
                                                                AnkleDataStru &right_ankle_joint_data) {
    // 将映射后的关节数据存储到对应的关节索引
    joint_data[4].pos_ = left_ankle_joint_data.pos_(1);  // Left Ankle Pitch
    joint_data[4].vel_ = left_ankle_joint_data.vel_(1);
    joint_data[4].tau_ = left_ankle_joint_data.tau_(1);
    joint_data[5].pos_ = left_ankle_joint_data.pos_(0);  // Left Ankle Roll
    joint_data[5].vel_ = left_ankle_joint_data.vel_(0);
    joint_data[5].tau_ = left_ankle_joint_data.tau_(0);

    joint_data[10].pos_ = right_ankle_joint_data.pos_(1);  // Right Ankle Pitch
    joint_data[10].vel_ = right_ankle_joint_data.vel_(1);
    joint_data[10].tau_ = right_ankle_joint_data.tau_(1);
    joint_data[11].pos_ = right_ankle_joint_data.pos_(0);  // Right Ankle Roll
    joint_data[11].vel_ = right_ankle_joint_data.vel_(0);
    joint_data[11].tau_ = right_ankle_joint_data.tau_(0);
}


void LeggedSystemHardware::readPublishAnkleDebugMessages() {
#if ANKLE_SOLVER_DEBUG_MODE_ENABLE
    real_input_motor_pos_pub_->publish(real_input_motor_pos_msg_);
    real_input_motor_vel_pub_->publish(real_input_motor_vel_msg_);
    real_input_motor_tau_pub_->publish(real_input_motor_tau_msg_);
    solver_mtj_input_motor_pos_pub_->publish(solver_mtj_input_motor_pos_msg_);
    solver_mtj_input_motor_vel_pub_->publish(solver_mtj_input_motor_vel_msg_);
    solver_mtj_input_motor_tau_pub_->publish(solver_mtj_input_motor_tau_msg_);
    solver_mtj_output_ankle_pos_pub_->publish(solver_mtj_output_ankle_pos_msg_);
    solver_mtj_output_ankle_vel_pub_->publish(solver_mtj_output_ankle_vel_msg_);
    solver_mtj_output_ankle_tau_pub_->publish(solver_mtj_output_ankle_tau_msg_);
#endif // ANKLE_SOLVER_DEBUG_MODE_ENABLE
}

void LeggedSystemHardware::readGetImuData() {
    // 从ROS2话题读取IMU数据
    std::lock_guard<std::mutex> imu_lock(imu_mutex_);

    // 在互斥锁保护下，将IMU数据复制到控制器可以直接访问的变量
    // 这样可以确保数据一致性，避免在读取过程中数据被更新
    imu_orientation_x_ = latest_imu_.orientation.x;
    imu_orientation_y_ = latest_imu_.orientation.y;
    imu_orientation_z_ = latest_imu_.orientation.z;
    imu_orientation_w_ = latest_imu_.orientation.w;

    imu_angular_velocity_x_ = latest_imu_.angular_velocity.x;
    imu_angular_velocity_y_ = latest_imu_.angular_velocity.y;
    imu_angular_velocity_z_ = latest_imu_.angular_velocity.z;

    imu_linear_acceleration_x_ = latest_imu_.linear_acceleration.x;
    imu_linear_acceleration_y_ = latest_imu_.linear_acceleration.y;
    imu_linear_acceleration_z_ = latest_imu_.linear_acceleration.z;

    // 打印 IMU 数据使用情况（每隔一段时间打印一次）
    // static int read_call_count = 0;
    // read_call_count++;
    // if (read_call_count % 1000 == 0) {  // 每1000次 read() 调用打印一次
    //     RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"),
    //         ">>> read() 已调用 %d 次 - 当前 IMU 数据: 姿态[%.3f,%.3f,%.3f,%.3f] 角速度[%.3f,%.3f,%.3f] 加速度[%.3f,%.3f,%.3f]",
    //         read_call_count,
    //         imu_orientation_x_, imu_orientation_y_, imu_orientation_z_, imu_orientation_w_,
    //         imu_angular_velocity_x_, imu_angular_velocity_y_, imu_angular_velocity_z_,
    //         imu_linear_acceleration_x_, imu_linear_acceleration_y_, imu_linear_acceleration_z_);
    // }
}

AnkleDataStru LeggedSystemHardware::writeCalculateAnkleMotorData(std::string ankle_name,
                                                                MotorDataStru &roll_joint_data,
                                                                MotorDataStru &pitch_joint_data) {
    // 将实际关节角度转换为解算器关节角度
    double roll_joint_tau = roll_joint_data.kp_ * (roll_joint_data.posDes_ - roll_joint_data.pos_) \
                                + roll_joint_data.kd_ * (roll_joint_data.velDes_ - roll_joint_data.vel_) \
                                + roll_joint_data.ff_;
    double pitch_joint_tau = pitch_joint_data.kp_ * (pitch_joint_data.posDes_ - pitch_joint_data.pos_) \
                                + pitch_joint_data.kd_ * (pitch_joint_data.velDes_ - pitch_joint_data.vel_) \
                                + pitch_joint_data.ff_;
    Eigen::Vector2d ankle_pos = {roll_joint_data.pos_, pitch_joint_data.pos_ - ankle_pitch_pos_offset_};  // Roll, Pitch
    Eigen::Vector2d ankle_vel = {roll_joint_data.vel_, pitch_joint_data.vel_};
    Eigen::Vector2d ankle_tau = {roll_joint_tau, pitch_joint_tau};

    // 初始化电机角度、速度、扭矩
    AnkleDataStru ankle_motor_data = AnkleDataStru();

    // 通过踝关节解算器计算电机角度
    if (ankle_name == "left") {
        std::tie(ankle_motor_data.pos_, ankle_motor_data.vel_, ankle_motor_data.tau_)
            = left_ankle_solver_->jointToMotor(ankle_pos, ankle_vel, ankle_tau);
    } else if (ankle_name == "right") {
        std::tie(ankle_motor_data.pos_, ankle_motor_data.vel_, ankle_motor_data.tau_)
            = right_ankle_solver_->jointToMotor(ankle_pos, ankle_vel, ankle_tau);
    }

#if ANKLE_SOLVER_DEBUG_MODE_ENABLE
    // 获取解算器输出的电机信息
    if (ankle_name == "left") {
        solver_jtm_output_motor_pos_msg_.data[0] = ankle_motor_data.pos_(0) * RAD_TO_DEG;
        solver_jtm_output_motor_pos_msg_.data[1] = ankle_motor_data.pos_(1) * RAD_TO_DEG;
        solver_jtm_output_motor_vel_msg_.data[0] = ankle_motor_data.vel_(0) * RAD_TO_DEG;
        solver_jtm_output_motor_vel_msg_.data[1] = ankle_motor_data.vel_(1) * RAD_TO_DEG;
        solver_jtm_output_motor_tau_msg_.data[0] = ankle_motor_data.tau_(0);
        solver_jtm_output_motor_tau_msg_.data[1] = ankle_motor_data.tau_(1);
    } else if (ankle_name == "right") {
        solver_jtm_output_motor_pos_msg_.data[2] = ankle_motor_data.pos_(0) * RAD_TO_DEG;
        solver_jtm_output_motor_pos_msg_.data[3] = ankle_motor_data.pos_(1) * RAD_TO_DEG;
        solver_jtm_output_motor_vel_msg_.data[2] = ankle_motor_data.vel_(0) * RAD_TO_DEG;
        solver_jtm_output_motor_vel_msg_.data[3] = ankle_motor_data.vel_(1) * RAD_TO_DEG;
        solver_jtm_output_motor_tau_msg_.data[2] = ankle_motor_data.tau_(0);
        solver_jtm_output_motor_tau_msg_.data[3] = ankle_motor_data.tau_(1);
    }
#endif // ANKLE_SOLVER_DEBUG_MODE_ENABLE

    // 将解算器电机角度转换为实际电机角度
    if (ankle_name == "left") {
        ankle_motor_data.pos_(0) -= left_ankle_left_motor_pos_offset_;
        ankle_motor_data.pos_(1) -= left_ankle_right_motor_pos_offset_;
    } else if (ankle_name == "right") {
        ankle_motor_data.pos_(0) -= right_ankle_left_motor_pos_offset_;
        ankle_motor_data.pos_(1) -= right_ankle_right_motor_pos_offset_;
    }

#if ANKLE_SOLVER_DEBUG_MODE_ENABLE
    // 获取实际电机信息
    if (ankle_name == "left") {
        real_output_motor_pos_msg_.data[0] = ankle_motor_data.pos_(0) * RAD_TO_DEG;
        real_output_motor_pos_msg_.data[1] = ankle_motor_data.pos_(1) * RAD_TO_DEG;
        real_output_motor_vel_msg_.data[0] = ankle_motor_data.vel_(0) * RAD_TO_DEG;
        real_output_motor_vel_msg_.data[1] = ankle_motor_data.vel_(1) * RAD_TO_DEG;
        real_output_motor_tau_msg_.data[0] = ankle_motor_data.tau_(0);
        real_output_motor_tau_msg_.data[1] = ankle_motor_data.tau_(1);
    } else if (ankle_name == "right") {
        real_output_motor_pos_msg_.data[2] = ankle_motor_data.pos_(0) * RAD_TO_DEG;
        real_output_motor_pos_msg_.data[3] = ankle_motor_data.pos_(1) * RAD_TO_DEG;
        real_output_motor_vel_msg_.data[2] = ankle_motor_data.vel_(0) * RAD_TO_DEG;
        real_output_motor_vel_msg_.data[3] = ankle_motor_data.vel_(1) * RAD_TO_DEG;
        real_output_motor_tau_msg_.data[2] = ankle_motor_data.tau_(0);
        real_output_motor_tau_msg_.data[3] = ankle_motor_data.tau_(1);
    }
#endif // ANKLE_SOLVER_DEBUG_MODE_ENABLE

    return ankle_motor_data;
}




// 检查单个踝关节电机限位的辅助函数
bool LeggedSystemHardware::writeCheckSingleAnkleMotorLimits(const std::string& motor_index,
                                                        double motor_pos,
                                                        double motor_vel,
                                                        double motor_tau,
                                                        const JointParamsStru& joint_params,
                                                        const std::string& ankle_name) {
    // 位置限位检查
    if (motor_pos > joint_params.motor_pos_limit_positive_ || motor_pos < joint_params.motor_pos_limit_negative_) {
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "%s Ankle Motor %s exceeded motor position limit! Press button B to reset. "
                    "motorPos=%.3f, motorPosLimit=[%.3f, %.3f]",
                    ankle_name.c_str(), motor_index.c_str(),
                    motor_pos, joint_params.motor_pos_limit_negative_, joint_params.motor_pos_limit_positive_);
        return true;
    }

    // 速度限位检查
    if (std::abs(motor_vel) > joint_params.motor_vel_limit_) {
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "%s Ankle Motor %s exceeded motor velocity limit! Press button B to reset. "
                    "motorVel=%.3f, motorVelLimit=%.3f",
                    ankle_name.c_str(), motor_index.c_str(),
                    motor_vel, joint_params.motor_vel_limit_);
        return true;
    }

    // 扭矩限位检查
    if (std::abs(motor_tau) > joint_params.motor_tau_limit_) {
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "%s Ankle Motor %s exceeded motor torque limit! Press button B to reset. "
                    "motorTau=%.3f, motorTauLimit=%.3f",
                    ankle_name.c_str(), motor_index.c_str(),
                    motor_tau, joint_params.motor_tau_limit_);
        return true;
    }
    return false;
}

void LeggedSystemHardware::writePrintAnkleSolverDebugMessages(std::string ankle_name, AnkleDataStru ankle_motor_data, JointParamsStru joint_params) {
    std::string unit_type = "deg";
    double unit_transform;

    if (unit_type == "deg") {
        unit_transform = RAD_TO_DEG;
    } else if (unit_type == "rad") {
        unit_transform = 1.0;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"), "Invalid unit_transform: %s", unit_type.c_str());
        unit_transform = 1.0; // 默认值
    }

    if (ankle_name == "left") {
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "1: left_ankle_input_motor_data (AnkleSolverSpace):\t\t"
                    "pos: [%.3f, %.3f], "
                    "vel: [%.3f, %.3f], "
                    "tau: [%.3f, %.3f]",
                    (inputMotorData_[4].pos_ + left_ankle_left_motor_pos_offset_) * unit_transform, (inputMotorData_[5].pos_ + left_ankle_right_motor_pos_offset_) * unit_transform,
                    inputMotorData_[4].vel_ * unit_transform, inputMotorData_[5].vel_ * unit_transform,
                    inputMotorData_[4].tau_, inputMotorData_[5].tau_);
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "2: left_ankle_calculated_roll_pitch_data (AnkleSolverSpace):\t"
                    "pos: [%.3f, %.3f], "
                    "vel: [%.3f, %.3f], "
                    "tau: [%.3f, %.3f], "
                    "num_iterations: %d, isSuccess: %d, final_error: %.6f",
                    jointData_[5].pos_ * unit_transform, (jointData_[4].pos_ - ankle_pitch_pos_offset_) * unit_transform,
                    jointData_[5].vel_ * unit_transform, jointData_[4].vel_ * unit_transform,
                    jointData_[5].tau_, jointData_[4].tau_,
                    left_ankle_mtj_num_iterations_, left_ankle_mtj_is_success_, left_ankle_mtj_final_error_);
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "3: left_ankle_calculated_motor_data (AnkleSolverSpace):\t\t"
                    "pos: [%.3f, %.3f], "
                    "vel: [%.3f, %.3f], "
                    "tau: [%.3f, %.3f]",
                    (ankle_motor_data.pos_(0) + left_ankle_left_motor_pos_offset_) * unit_transform, (ankle_motor_data.pos_(1) + left_ankle_right_motor_pos_offset_) * unit_transform,
                    ankle_motor_data.vel_(0) * unit_transform, ankle_motor_data.vel_(1) * unit_transform,
                    ankle_motor_data.tau_(0), ankle_motor_data.tau_(1));

    } else if (ankle_name == "right") {
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "1: right_ankle_input_motor_data (AnkleSolverSpace):\t\t"
                    "pos: [%.3f, %.3f], "
                    "vel: [%.3f, %.3f], "
                    "tau: [%.3f, %.3f]",
                    (inputMotorData_[11].pos_ + right_ankle_left_motor_pos_offset_) * unit_transform, (inputMotorData_[10].pos_ + right_ankle_right_motor_pos_offset_) * unit_transform,
                    inputMotorData_[11].vel_ * unit_transform, inputMotorData_[10].vel_ * unit_transform,
                    inputMotorData_[11].tau_, inputMotorData_[10].tau_);
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "2: right_ankle_calculated_roll_pitch_data (AnkleSolverSpace):\t"
                    "pos: [%.3f, %.3f], "
                    "vel: [%.3f, %.3f], "
                    "tau: [%.3f, %.3f], "
                    "num_iterations: %d, isSuccess: %d, final_error: %.6f",
                    jointData_[11].pos_ * unit_transform, (jointData_[10].pos_ - ankle_pitch_pos_offset_) * unit_transform,
                    jointData_[11].vel_ * unit_transform, jointData_[10].vel_ * unit_transform,
                    jointData_[11].tau_, jointData_[10].tau_,
                    right_ankle_mtj_num_iterations_, right_ankle_mtj_is_success_, right_ankle_mtj_final_error_);
        RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"),
                    "3: right_ankle_calculated_motor_data (AnkleSolverSpace):\t\t"
                    "pos: [%.3f, %.3f], "
                    "vel: [%.3f, %.3f], "
                    "tau: [%.3f, %.3f]",
                    (ankle_motor_data.pos_(0) + right_ankle_left_motor_pos_offset_) * unit_transform, (ankle_motor_data.pos_(1) + right_ankle_right_motor_pos_offset_) * unit_transform,
                    ankle_motor_data.vel_(0) * unit_transform, ankle_motor_data.vel_(1) * unit_transform,
                    ankle_motor_data.tau_(0), ankle_motor_data.tau_(1));
    }
}

// 重置限位状态函数
void LeggedSystemHardware::writeJointLimitStatusReset() {
    writeJointLimitExceeded_ = false;
    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"),
                "Joint limit exceeded status has been reset. Normal operation resumed.");
}

void LeggedSystemHardware::writePublishAnkleDebugMessages() {
    // 发布解算器debug数据
#if ANKLE_SOLVER_DEBUG_MODE_ENABLE
    solver_jtm_output_motor_pos_pub_->publish(solver_jtm_output_motor_pos_msg_);
    solver_jtm_output_motor_vel_pub_->publish(solver_jtm_output_motor_vel_msg_);
    solver_jtm_output_motor_tau_pub_->publish(solver_jtm_output_motor_tau_msg_);
    real_output_motor_pos_pub_->publish(real_output_motor_pos_msg_);
    real_output_motor_vel_pub_->publish(real_output_motor_vel_msg_);
    real_output_motor_tau_pub_->publish(real_output_motor_tau_msg_);
#endif // ANKLE_SOLVER_DEBUG_MODE_ENABLE
}


}   // namespace legged