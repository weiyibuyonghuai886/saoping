#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <unordered_map>  // 添加哈希表头文件

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "Types.h"
#include "utilities.h"

// 添加custom_controller_msgs头文件
#include "custom_controller_msgs/msg/custom_control_command.hpp"

// 添加踝关节解算器头文件
#include "AnkleSolver.h"

#include <cstdio>

#include <csignal>
#include <unistd.h>
#include <iomanip>
#include "std_msgs/msg/float64.hpp"

#define NUM_MOTOR 30    // 电机数量(关节数量)
#define ANKLE_SOLVER_DEBUG_MODE_ENABLE 1    // 踝关节解算器调试信息开关
// Set pos, vel, tau, kp, kd (0x01 | 0x02 | 0x04 | 0x08 | 0x10)
#define CMD_MODE_TAU 0x19   // Set tau (0x04)
#define CMD_MODE_POS 0x19   // Set pos, vel, tau, kp, kd (0x01 | 0x02 | 0x04 | 0x08 | 0x10)
#define CMD_MODE_ALL 0x19   // Set pos, vel, tau, kp, kd (0x01 | 0x02 | 0x04 | 0x08 | 0x10)

namespace legged {

// 关节参数结构体
struct JointParamsStru {
  const char* jointName;         // 关节名称
  std::string motorIndex;        // 电机编号（使用字符串以匹配JointState消息）
  int jointIndex;                // 关节索引
  int directionMotor_;           // 电机方向（1或-1）
  float pos_limit_positive_;     // 正向位置限制（弧度）
  float pos_limit_negative_;     // 负向位置限制（弧度）
  float vel_limit_;              // 速度限制（rad/s）
  float tau_limit_;              // 扭矩限制（Nm）
  float biasMotor_;              // 电机位置偏置（弧度）
  bool isAnkleMotor_;            // 是否为踝关节电机
  int ankleSolverMotorIndex_;    // 踝关节解算器电机索引
  float motor_pos_limit_positive_;  // 脚踝电机正向位置限制（弧度）
  float motor_pos_limit_negative_;  // 脚踝电机负向位置限制（弧度）
  float motor_vel_limit_;           // 脚踝电机速度限制（rad/s）
  float motor_tau_limit_;           // 脚踝电机扭矩限制（Nm）
};

// 电机数据结构体(double类型)
struct MotorDataStru
{
    double pos_, vel_, tau_;                        // state
    double posDes_, velDes_, kp_, kd_, ff_;         // command
};

// 关节名称及方向偏置结构体
struct JointNameParamsStru {
    std::string name;           // 关节名称
    int directionMotor_;        // 电机方向（1或-1）
    double biasMotor_;          // 电机位置偏置（弧度）
    double pos_limit_positive_; // 正向位置限制（弧度）
    double pos_limit_negative_; // 负向位置限制（弧度）
    double vel_limit_;          // 速度限制（rad/s）
    double tau_limit_;          // 扭矩限制（Nm）
    bool ignore_limit_check_;   // 是否忽略超限检测
};

using Clock = std::chrono::high_resolution_clock; // 使用高精度时钟
using Duration = std::chrono::duration<double>; // 使用double类型的持续时间

class LeggedSystemHardware : public hardware_interface::SystemInterface {

public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ros2_control 基础框架
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    int joint_remapping_[NUM_MOTOR] = {
        4, 7, 3, 14, 10, 1, // left leg
        5, 19, 23, 11, 9, 6, // right leg
        8, // waist
        0, 15, 20, 24, 22, 2, // left arm
        13, 21, 18, 17, 12,16  // right arm
    };
    std::vector<JointNameParamsStru> all_joint_names_;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // on_init 函数相关成员变量
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::shared_ptr<rclcpp::Node> node_;    // ros2节点

    // 关节参数数组成员变量
    JointParamsStru jointParams_[NUM_MOTOR];   // 关节参数数组
    std::unordered_map<std::string, int> motorIndexToJointParamsIndex_;  // 哈希表用于快速查找motorIndex(字符串)对应的jointParams索引
    void initializeJointParams();
    // void printJointParams();

    // 关节名称参数加载
    void loadJointNameParams();

    // 踝关节解算器成员变量
    std::unique_ptr<HumanoidParallelAnkleSolver> left_ankle_solver_;
    std::unique_ptr<HumanoidParallelAnkleSolver> right_ankle_solver_;
    double left_ankle_left_motor_pos_offset_;
    double left_ankle_right_motor_pos_offset_;
    double right_ankle_left_motor_pos_offset_;
    double right_ankle_right_motor_pos_offset_;
    double ankle_pitch_pos_offset_;
    void initializeAnkleSolvers();

    // 移动平均滤波器
    // 左脚踝电机速度滤波
    std::unique_ptr<MovingAverageFilter> left_ankle_left_motor_vel_filter_;
    std::unique_ptr<MovingAverageFilter> left_ankle_right_motor_vel_filter_;
    // 右脚踝电机速度滤波
    std::unique_ptr<MovingAverageFilter> right_ankle_left_motor_vel_filter_;
    std::unique_ptr<MovingAverageFilter> right_ankle_right_motor_vel_filter_;
    // 腿部关节速度数据滤波
    std::unique_ptr<MovingAverageFilter> leg_l1_joint_vel_filter_;
    std::unique_ptr<MovingAverageFilter> leg_l2_joint_vel_filter_;
    std::unique_ptr<MovingAverageFilter> leg_l3_joint_vel_filter_;
    std::unique_ptr<MovingAverageFilter> leg_l4_joint_vel_filter_;
    std::unique_ptr<MovingAverageFilter> leg_r1_joint_vel_filter_;
    std::unique_ptr<MovingAverageFilter> leg_r2_joint_vel_filter_;
    std::unique_ptr<MovingAverageFilter> leg_r3_joint_vel_filter_;
    std::unique_ptr<MovingAverageFilter> leg_r4_joint_vel_filter_;
    void initializeJointVelFilters();

    // 踝关节解算器调试消息
    // 用于调试脚踝解算器的数据
    std_msgs::msg::Float64MultiArray real_input_motor_pos_msg_, real_input_motor_vel_msg_, real_input_motor_tau_msg_;   // 真实电机输入
    std_msgs::msg::Float64MultiArray solver_mtj_input_motor_pos_msg_, solver_mtj_input_motor_vel_msg_, solver_mtj_input_motor_tau_msg_;  // motor to joint 输入
    std_msgs::msg::Float64MultiArray solver_mtj_output_ankle_pos_msg_, solver_mtj_output_ankle_vel_msg_, solver_mtj_output_ankle_tau_msg_;  // motor to joint 输出
    std_msgs::msg::Float64MultiArray solver_jtm_output_motor_pos_msg_, solver_jtm_output_motor_vel_msg_, solver_jtm_output_motor_tau_msg_;  // joint to motor 输出
    std_msgs::msg::Float64MultiArray real_output_motor_pos_msg_, real_output_motor_vel_msg_, real_output_motor_tau_msg_;  // 真实电机输出
    std_msgs::msg::Float64 read_time_msg_, read_period_msg_, write_time_msg_, write_period_msg_;  // 读写时间和周期
    std_msgs::msg::Float64 cal_read_time_msg_, cal_write_time_msg_;  // 计算读取和写入时间消息
    void initializeAnkleDebugMessages();
    //脚踝解算器数据发布者
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr real_input_motor_pos_pub_, real_input_motor_vel_pub_, real_input_motor_tau_pub_; // 真实电机输入
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr solver_mtj_input_motor_pos_pub_, solver_mtj_input_motor_vel_pub_, solver_mtj_input_motor_tau_pub_; // motor to joint 输入
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr solver_mtj_output_ankle_pos_pub_, solver_mtj_output_ankle_vel_pub_, solver_mtj_output_ankle_tau_pub_; // motor to joint 输出
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr solver_jtm_output_motor_pos_pub_, solver_jtm_output_motor_vel_pub_, solver_jtm_output_motor_tau_pub_; // joint to motor 输出
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr real_output_motor_pos_pub_, real_output_motor_vel_pub_, real_output_motor_tau_pub_; // 真实电机输出
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr read_time_pub_, read_period_pub_, write_time_pub_, write_period_pub_;  // 读写时间和周期
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cal_read_time_pub_, cal_write_time_pub_;  // 计算读取和写入时间发布者
    void initializeAnkleDebugPublishers();

    // 关节数据
    MotorDataStru inputMotorData_[NUM_MOTOR]{};
    MotorDataStru jointData_[NUM_MOTOR]{};

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // export_state_interfaces 函数相关成员变量
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // IMU数据变量 - 用于控制器直接访问，确保线程安全
    double imu_orientation_x_ = 0.0;
    double imu_orientation_y_ = 0.0;
    double imu_orientation_z_ = 0.0;
    double imu_orientation_w_ = 1.0;

    double imu_angular_velocity_x_ = 0.0;
    double imu_angular_velocity_y_ = 0.0;
    double imu_angular_velocity_z_ = 0.0;

    double imu_linear_acceleration_x_ = 0.0;
    double imu_linear_acceleration_y_ = 0.0;
    double imu_linear_acceleration_z_ = 0.0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // on_activate 函数相关成员变量
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 硬件通信相关发布者和订阅者
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;                    // IMU订阅者
    rclcpp::Publisher<custom_controller_msgs::msg::CustomControlCommand>::SharedPtr custom_cmd_pub_;  // 自定义控制命令发布者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;        // 关节状态反馈发布者
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;     // 关节状态订阅者（接收硬件反馈）
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr reset_joint_limit_status_sub_;     // 重置关节限位状态订阅者
    void activateHardwareCommunication();

    // 线程执行器
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;

    // 线程安全的数据存储
    std::mutex imu_mutex_;
    sensor_msgs::msg::Imu latest_imu_;
    std::mutex joint_state_mutex_;
    sensor_msgs::msg::JointState latest_joint_state_;

    // 回调函数
    void callbackImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void callbackJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
    void callbackWriteJointLimitStatusReset(const std_msgs::msg::Float32::SharedPtr msg);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // read 函数相关成员变量
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int left_ankle_mtj_num_iterations_ = 0; // 左脚踝电机到关节的迭代次数
    bool left_ankle_mtj_is_success_ = false; // 左脚踝电机到关节的迭代是否成功
    double left_ankle_mtj_final_error_ = 0.0; // 左脚踝电机到关节的最终误差
    int right_ankle_mtj_num_iterations_ = 0; // 右脚踝电机到关节的迭代次数
    bool right_ankle_mtj_is_success_ = false; // 右脚踝电机到关节的迭代是否成功
    double right_ankle_mtj_final_error_ = 0.0; // 右脚踝电机到关节的最终误差

    // 获取踝关节电机数据
    void readMotorData(MotorDataStru joint_data[NUM_MOTOR],
                        AnkleDataStru &left_ankle_motor_data,
                        AnkleDataStru &right_ankle_motor_data,
                        bool &left_ankle_data_ready,
                        bool &right_ankle_data_ready);

    // 应用滤波器
    void readApplyFilters(MotorDataStru joint_data[NUM_MOTOR],
                            AnkleDataStru &left_ankle_motor_data,
                            AnkleDataStru &right_ankle_motor_data);

    // 计算踝关节脚踝数据 motor (left, right) to joint (roll, pitch): 基于电机采样数据计算踝关节数据
    AnkleDataStru readCalculateAnkleRollPitchFromMotor(std::string ankle_name, AnkleDataStru &ankle_motor_data);

    // 更新踝关节数据
    void readUpdateAnkleRollPitchToJointData(MotorDataStru joint_data[NUM_MOTOR],
                                                AnkleDataStru &left_ankle_joint_data,
                                                AnkleDataStru &right_ankle_joint_data);

    // 发布踝关节解算器调试消息
    void readPublishAnkleDebugMessages();

    // 获取IMU数据
    void readGetImuData();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // write 函数相关成员变量
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 计算踝关节电机数据 joint (roll, pitch) to motor (left, right): 基于关节数据计算电机下发数据
    AnkleDataStru writeCalculateAnkleMotorData(std::string ankle_name,
                                            MotorDataStru &roll_joint_data,
                                            MotorDataStru &pitch_joint_data);

    // 关节限位相关成员变量
    bool writeJointLimitExceeded_ = false;
    // 检查单个踝关节电机限位的辅助函数
    bool writeCheckSingleAnkleMotorLimits(const std::string& motor_index,
                                            double motor_pos,
                                            double motor_vel,
                                            double motor_tau,
                                            const JointParamsStru& joint_params,
                                            const std::string& ankle_name);
    // 报错时打印踝关节解算器调试消息
    void writePrintAnkleSolverDebugMessages(std::string ankle_name,
                                            AnkleDataStru ankle_motor_data,
                                            JointParamsStru joint_params);
    void writeJointLimitStatusReset();


    // 踝关节解算器调试消息发布
    void writePublishAnkleDebugMessages();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 未使用的参数
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    tf2::Quaternion q;

    // imu 通过topic传递
    sensor_msgs::msg::Imu imuMsg_;
    sensor_msgs::msg::Imu yesenceIMU_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr yesenseImuSub_;
};

}