/*
 * NOTE:
 * 本代码只包含 ros2_control 的基本框架函数,自定义函数均在 LeggedSystemHelperFunctions.cpp 中实现
 * 1. 初始化函数在on_init()中调用
 * 2. 导出状态和命令接口在export_state_interfaces()和export_command_interfaces()中调用
 * 3. 激活和停用硬件在on_activate()和on_deactivate()中调用
 * 4. 读取硬件状态在read()中调用
 * 5. 写入硬件命令在write()中调用
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
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "utilities.h"
#include <boost/smart_ptr/detail/sp_counted_impl.hpp>

namespace legged {

template <int row_>
using Vector = Eigen::Matrix<double, row_, 1>;

hardware_interface::CallbackReturn LeggedSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    // 解析不到urdf中硬件信息，报错
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 创建订阅节点， ROS2话题相关初始化操作
    this->node_ =  std::make_shared<rclcpp::Node>("hardware_node");

    // 打印关节信息
    for (auto joint : info.joints) {
        RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"),
        "Found %s successfully!", joint.name.c_str());
    }

    // 加载关节名称参数
    loadJointNameParams();
    // 初始化关节参数数组
    initializeJointParams();
    // 初始化踝关节解算器
    // initializeAnkleSolvers();
    // 初始化关节速度滤波器
    // initializeJointVelFilters();
    // 初始化脚踝解算器调试消息
    // initializeAnkleDebugMessages();
    // 初始化脚踝解算器调试发布者
    // initializeAnkleDebugPublishers();

    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "LeggedSystemHardware initialized successfully");

    return hardware_interface::CallbackReturn::SUCCESS;
}


// 导出接口
// 控制器直接对commmand_interfaces_赋值和读取state_interfaces_来完成与硬件组件的交互。
std::vector<hardware_interface::StateInterface> LeggedSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Exporting state interfaces...");
    // 这里的joints顺序是按照urdf中ros2_control下的joint顺序
    // l leg rleg waist pitch roll yaw head yaw pitch l arm r arm
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &jointData_[i].pos_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &jointData_[i].vel_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &jointData_[i].tau_));
    }
    // 在 export_state_interfaces() 最后添加：
    for (size_t i = 0; i < info_.sensors.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[i].name, "orientation.x", &imu_orientation_x_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[i].name, "orientation.y", &imu_orientation_y_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[i].name, "orientation.z", &imu_orientation_z_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[i].name, "orientation.w", &imu_orientation_w_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[i].name, "angular_velocity.x", &imu_angular_velocity_x_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[i].name, "angular_velocity.y", &imu_angular_velocity_y_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[i].name, "angular_velocity.z", &imu_angular_velocity_z_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[i].name, "linear_acceleration.x", &imu_linear_acceleration_x_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[i].name, "linear_acceleration.y", &imu_linear_acceleration_y_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[i].name, "linear_acceleration.z", &imu_linear_acceleration_z_));
    }

    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "State interfaces exported successfully");
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LeggedSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Exporting command interfaces...");
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &jointData_[i].posDes_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &jointData_[i].velDes_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &jointData_[i].ff_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "Kp", &jointData_[i].kp_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "Kd", &jointData_[i].kd_));

        RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "%s", command_interfaces[i].get_name().c_str());
    }

    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Command interfaces exported successfully");
    return command_interfaces;
}

hardware_interface::CallbackReturn LeggedSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Activating ...please wait...");

    // 设置硬件通信
    activateHardwareCommunication();

    // 创建线程执行器
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(this->node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Activation completed");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LeggedSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Deactivating ...please wait...");

    // 类似 析构函数的功能
    if (executor_thread_.joinable()) {
        executor_->cancel();
        executor_thread_.join();
    }

    RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Deactivation completed");
    return hardware_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NOTE: 读取机器人硬件状态函数
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hardware_interface::return_type LeggedSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // 获取运行时间相关数据（仅当发布者已初始化时发布）
    if (read_time_pub_) {
        read_time_msg_.data = time.seconds();
        read_time_pub_->publish(read_time_msg_);
    }
    if (read_period_pub_) {
        read_period_msg_.data = period.seconds();
        read_period_pub_->publish(read_period_msg_);
    }

    // 发布计算的读取时间
    Clock::time_point read_start_time = Clock::now();

    // 临时存储脚踝电机数据
    AnkleDataStru left_ankle_motor_data = AnkleDataStru(), right_ankle_motor_data = AnkleDataStru();    // 脚踝电机数据(从电机读取)
    bool left_ankle_motor_data_ready = false, right_ankle_motor_data_ready = false;         // 脚踝电机数据是否准备好
    // AnkleDataStru left_ankle_joint_data = AnkleDataStru(), right_ankle_joint_data = AnkleDataStru();    // 脚踝关节数据(通过踝关节解算器计算)

    // 重置关节限位标志
    writeJointLimitExceeded_ = false;

    // 读取脚踝电机数据(此时jointData_内脚踝信息为空)
    // 根据名字匹配，确保 jointData_ 的填充顺序与 all_joint_names_ 一致
    for (size_t i = 0; i < NUM_MOTOR; ++i) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        const auto& jointNameParam = all_joint_names_[i]; // 从all_joint_names_获取方向和偏置

        // 对于手臂后三个关节和颈部关节，直接填充为0
        // 左臂后三个: arm_joint_5(18), arm_joint_6(19), arm_joint_7(20)
        // 右臂后三个: arm_joint_12(25), arm_joint_13(26), arm_joint_14(27)
        // 颈部: neck_yaw(28), neck_pitch(29)
        if (i == 18 || i == 19 || i == 20 ||  // 左臂后三个
            i == 25 || i == 26 || i == 27 ||  // 右臂后三个
            i == 28 || i == 29) {              // 颈部
            jointData_[i].pos_ = 0.0;
            jointData_[i].vel_ = 0.0;
            jointData_[i].tau_ = 0.0;
            continue;
        }

        // 在 latest_joint_state_.name 中查找当前关节名
        auto it = std::find(latest_joint_state_.name.begin(),
                           latest_joint_state_.name.end(),
                           jointNameParam.name);

        if (it != latest_joint_state_.name.end()) {
            // 找到匹配的关节名，计算其索引
            size_t joint_index = std::distance(latest_joint_state_.name.begin(), it);
            jointData_[i].pos_ = latest_joint_state_.position[joint_index] * jointNameParam.directionMotor_ + jointNameParam.biasMotor_;
            jointData_[i].vel_ = latest_joint_state_.velocity[joint_index] * jointNameParam.directionMotor_;
            jointData_[i].tau_ = latest_joint_state_.effort[joint_index] * jointNameParam.directionMotor_;

            // 检查关节限位（检查从硬件读取的实际值）
            // 如果该关节设置为忽略超限检测，则跳过检查
            if (!jointNameParam.ignore_limit_check_) {
                bool limit_exceeded = false;

                // 检查位置限位（跳过占位符关节，即限位为0的关节）
                if (jointNameParam.pos_limit_positive_ != 0.0 || jointNameParam.pos_limit_negative_ != 0.0) {
                    if (jointData_[i].pos_ > jointNameParam.pos_limit_positive_ ||
                        jointData_[i].pos_ < jointNameParam.pos_limit_negative_) {
                        limit_exceeded = true;
                        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("LeggedSystemHardware"),
                                            *node_->get_clock(), 1000,
                                            "Joint '%s' position limit exceeded: pos=%.4f, limits=[%.4f, %.4f]",
                                            jointNameParam.name.c_str(), jointData_[i].pos_,
                                            jointNameParam.pos_limit_negative_, jointNameParam.pos_limit_positive_);
                    }
                }

                // 检查速度限位
                if (jointNameParam.vel_limit_ != 0.0) {
                    if (std::abs(jointData_[i].vel_) > jointNameParam.vel_limit_) {
                        limit_exceeded = true;
                        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("LeggedSystemHardware"),
                                            *node_->get_clock(), 1000,
                                            "Joint '%s' velocity limit exceeded: vel=%.4f, limit=%.4f",
                                            jointNameParam.name.c_str(), jointData_[i].vel_, jointNameParam.vel_limit_);
                    }
                }

                // 检查力矩限位
                if (jointNameParam.tau_limit_ != 0.0) {
                    if (std::abs(jointData_[i].tau_) > jointNameParam.tau_limit_) {
                        limit_exceeded = true;
                        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("LeggedSystemHardware"),
                                            *node_->get_clock(), 1000,
                                            "Joint '%s' torque limit exceeded: tau=%.4f, limit=%.4f",
                                            jointNameParam.name.c_str(), jointData_[i].tau_, jointNameParam.tau_limit_);
                    }
                }

                // 如果该关节超限，设置全局标志
                if (limit_exceeded) {
                    writeJointLimitExceeded_ = true;
                }
            }

            // std::cerr << "joint_name: " << *it << ", all_joint_names_[" << i << "], joint_index: " << joint_index << std::endl;
        } else {
            // 未找到匹配的关节名，记录警告
            std::string joint_names_str;
            for (size_t j = 0; j < latest_joint_state_.name.size(); ++j) {
                if (j > 0) joint_names_str += ", ";
                joint_names_str += latest_joint_state_.name[j];
            }
            // RCLCPP_WARN_THROTTLE(rclcpp::get_logger("LeggedSystemHardware"),
            //                     *node_->get_clock(), 1000,
            //                     "Joint '%s' not found in latest_joint_state_.name. Available joints: [%s]",
            //                     jointNameParam.name.c_str(),
            //                     joint_names_str.c_str());
        }
        // jointData_[i].pos_ = low_state_.position[i];
        // jointData_[i].vel_ = low_state_.velocity[i];
        // jointData_[i].tau_ = low_state_.effort[i];
        // RCLCPP_INFO(rclcpp::get_logger("LeggedSystemHardware"), "Joint direction: %d, Joint bias: %f, pos=%.3f, vel=%.3f, tau=%.3f", jointNameParam.directionMotor_, jointNameParam.biasMotor_, jointData_[i].pos_, jointData_[i].vel_, jointData_[i].tau_);
      }

    // 应用滤波器
    // readApplyFilters(jointData_, left_ankle_motor_data, right_ankle_motor_data);    // 此时的left_ankle_motor_data和right_ankle_motor_data在原电机空间下

    // 通过踝关节解算器计算脚踝关节角度
    // if (left_ankle_motor_data_ready) {
    //     left_ankle_joint_data = readCalculateAnkleRollPitchFromMotor("left", left_ankle_motor_data);    // 此时的left_ankle_motor_data在踝关节解算器空间下
    // }

    // 通过踝关节解算器计算脚踝关节角度
    // if (right_ankle_motor_data_ready) {
    //     right_ankle_joint_data = readCalculateAnkleRollPitchFromMotor("right", right_ankle_motor_data);    // 此时的right_ankle_motor_data在踝关节解算器空间下
    // }

    // 将 jointData_ 拷贝给 inputMotorData_ 作为数据分析
    // for (int i = 0; i < NUM_MOTOR; i++) {
    //     inputMotorData_[i] = jointData_[i];
    // }

    // 将脚踝关节角度更新到关节数据中(此时jointData_内的脚踝信息为roll, pitch的信息)
    // readUpdateAnkleRollPitchToJointData(jointData_, left_ankle_joint_data, right_ankle_joint_data);    // 此时的left_ankle_joint_data和right_ankle_joint_data在原关节空间下

    // 发布解算器debug数据
    // readPublishAnkleDebugMessages();

    // 从ROS2话题读取IMU数据
    readGetImuData();

    // 发布计算的读取时间（仅当发布者已初始化时）
    if (cal_read_time_pub_) {
        Clock::time_point read_end_time = Clock::now();
        Duration read_duration = std::chrono::duration_cast<Duration>(read_end_time - read_start_time);
        cal_read_time_msg_.data = read_duration.count();
        cal_read_time_pub_->publish(cal_read_time_msg_);
    }

    return hardware_interface::return_type::OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NOTE: 写入机器人硬件状态函数
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hardware_interface::return_type LeggedSystemHardware::write( const rclcpp::Time &time, const rclcpp::Duration &period) {

    // 发布写入时间（仅当发布者已初始化时）
    if (write_time_pub_) {
        write_time_msg_.data = time.seconds();
        write_time_pub_->publish(write_time_msg_);
    }
    if (write_period_pub_) {
        write_period_msg_.data = period.seconds();
        write_period_pub_->publish(write_period_msg_);
    }

    // 发布计算的读写时间
    Clock::time_point write_start_time = Clock::now();

    // 创建CustomControlCommand消息
    auto custom_cmd_msg = custom_controller_msgs::msg::CustomControlCommand();
    custom_cmd_msg.header.stamp = time;
    custom_cmd_msg.header.frame_id = "base_link";

    // 固定的32个关节名称顺序
    std::vector<std::string> joint_names = {
        "leg_joint_1", "leg_joint_2", "leg_joint_3", "leg_joint_4", "leg_joint_5", "leg_joint_6",
        "leg_joint_7", "leg_joint_8", "leg_joint_9", "leg_joint_10", "leg_joint_11", "leg_joint_12",
        "leg_joint_13", "leg_joint_14",  // 占位符
        "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5", "arm_joint_6", "arm_joint_7",
        "arm_joint_8", "arm_joint_9", "arm_joint_10", "arm_joint_11", "arm_joint_12", "arm_joint_13", "arm_joint_14",
        "waist_yaw", "waist_pitch", "neck_yaw", "neck_pitch"  // 腰部和头部
    };

    // 初始化32个关节的数据数组
    custom_cmd_msg.joint_names = joint_names;
    custom_cmd_msg.position.resize(32, 0.0);
    custom_cmd_msg.velocity.resize(32, 0.0);
    custom_cmd_msg.effort.resize(32, 0.0);
    custom_cmd_msg.kp.resize(32, 0.0);
    custom_cmd_msg.kd.resize(32, 0.0);

    // 计算脚踝电机数据（暂时禁用脚踝解算器，使用直接位置控制）
    // AnkleDataStru left_ankle_motor_data = AnkleDataStru(), right_ankle_motor_data = AnkleDataStru();
    // left_ankle_motor_data = writeCalculateAnkleMotorData("left", jointData_[5], jointData_[4]);
    // right_ankle_motor_data = writeCalculateAnkleMotorData("right", jointData_[11], jointData_[10]);

    // 按固定顺序填充关节数据
    for (size_t i = 0; i < joint_names.size(); i++) {
        const std::string& joint_name = joint_names[i];

        // 在 all_joint_names_ 中查找匹配的关节名
        auto it = std::find_if(all_joint_names_.begin(), all_joint_names_.end(),
                               [&joint_name](const JointNameParamsStru& jnp) {
                                   return jnp.name == joint_name;
                               });

        if (it == all_joint_names_.end()) {
            // all_joint_names_ 中没有找到该关节，填充0.0
            // 例如: leg_joint_7, leg_joint_13, leg_joint_14, waist_yaw, neck_yaw, neck_pitch 等
            custom_cmd_msg.position[i] = 0.0;
            custom_cmd_msg.velocity[i] = 0.0;
            custom_cmd_msg.effort[i] = 0.0;
            custom_cmd_msg.kp[i] = 0.0;
            custom_cmd_msg.kd[i] = 0.0;
            continue;
        }

        // 找到匹配的关节，计算其在 all_joint_names_ 中的索引
        size_t joint_index = std::distance(all_joint_names_.begin(), it);
        const auto& jointNameParam = all_joint_names_[joint_index];  // 获取方向和偏置
        const auto& joint_data = jointData_[joint_index];             // 获取关节数据

        // 使用位置控制模式逻辑填充数据
        custom_cmd_msg.position[i] = (joint_data.posDes_ - jointNameParam.biasMotor_) * jointNameParam.directionMotor_;
        custom_cmd_msg.velocity[i] = joint_data.velDes_ * jointNameParam.directionMotor_;
        custom_cmd_msg.effort[i] = joint_data.ff_ * jointNameParam.directionMotor_;
        custom_cmd_msg.kp[i] = joint_data.kp_;
        custom_cmd_msg.kd[i] = joint_data.kd_;

        // 如果在read()中检测到关节限位超限，设置为安全值
        if (writeJointLimitExceeded_) {
            custom_cmd_msg.position[i] = 0.0;
            custom_cmd_msg.velocity[i] = 0.0;
            custom_cmd_msg.effort[i] = 0.0;
            custom_cmd_msg.kp[i] = 0.0;
            custom_cmd_msg.kd[i] = 0.3;
        }
    }

    // 发布解算器debug数据
    // writePublishAnkleDebugMessages();

    // 发布自定义控制命令（应该已在 on_activate 中初始化）
    if (custom_cmd_pub_) {
        custom_cmd_pub_->publish(custom_cmd_msg);
    }

    // 发布计算的写入时间（仅当发布者已初始化时）
    if (cal_write_time_pub_) {
        Clock::time_point write_end_time = Clock::now();
        Duration write_duration = std::chrono::duration_cast<Duration>(write_end_time - write_start_time);
        cal_write_time_msg_.data = write_duration.count();
        cal_write_time_pub_->publish(cal_write_time_msg_);
    }

    return hardware_interface::return_type::OK;
}

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::LeggedSystemHardware, hardware_interface::SystemInterface)

