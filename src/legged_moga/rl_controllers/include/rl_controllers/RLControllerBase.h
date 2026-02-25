#pragma once

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <memory>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <urdf/model.h>
#include <semantic_components/imu_sensor.hpp>

#include "rl_controllers/Types.h"
#include "rl_controllers/HybridJointHandle.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm> // 确保包含此头文件以使用 std::min 和 std::max
#include <map>       // 用于数据记录容器
template <typename T>
T clamp(T value, T min, T max) {
    return std::max(min, std::min(value, max));
}

namespace legged {

struct RLRobotCfg {
  struct ControlCfg {
    std::map<std::string, float> stiffness;
    std::map<std::string, float> damping;
    float actionScale;
    int decimation;
    float user_torque_limit;
    float user_power_limit;
    float cycle_time;
  };

  struct InitState {
    // default joint angles - MOGA legs
    scalar_t left_hip_pitch;
    scalar_t left_hip_roll;
    scalar_t left_hip_yaw;
    scalar_t left_knee_pitch;
    scalar_t left_ankle_pitch;
    scalar_t left_ankle_roll;
    scalar_t right_hip_pitch;
    scalar_t right_hip_roll;
    scalar_t right_hip_yaw;
    scalar_t right_knee_pitch;
    scalar_t right_ankle_pitch;
    scalar_t right_ankle_roll;
  };
  struct armInitState{
    // MOGA arm joints
    scalar_t left_shoulder_pitch;
    scalar_t left_shoulder_roll;
    scalar_t left_shoulder_yaw;
    scalar_t left_elbow_pitch;
    scalar_t left_elbow_yaw;
    scalar_t left_wrist_pitch;
    scalar_t left_wrist_roll;
    scalar_t right_shoulder_pitch;
    scalar_t right_shoulder_roll;
    scalar_t right_shoulder_yaw;
    scalar_t right_elbow_pitch;
    scalar_t right_elbow_yaw;
    scalar_t right_wrist_pitch;
    scalar_t right_wrist_roll;
  };

  struct waistInitState{
    scalar_t waist_yaw;
    scalar_t waist_pitch;
  };
  struct headInitState{
    scalar_t head_yaw;
    scalar_t head_pitch;
  };

  struct ObsScales {
    scalar_t linVel;
    scalar_t angVel;
    scalar_t dofPos;
    scalar_t dofVel;
    scalar_t quat;
    scalar_t heightMeasurements;
  };

  bool encoder_nomalize;

  scalar_t clipActions;
  scalar_t clipObs;

  InitState initState;
  armInitState arminitState;
  waistInitState waistinitState;
  headInitState headInitState;
  ObsScales obsScales;
  ControlCfg controlCfg;
};

struct JointState
  {
    // MOGA Left Leg
    scalar_t left_hip_pitch;
    scalar_t left_hip_roll;
    scalar_t left_hip_yaw;
    scalar_t left_knee_pitch;
    scalar_t left_ankle_pitch;
    scalar_t left_ankle_roll;
    // MOGA Right Leg
    scalar_t right_hip_pitch;
    scalar_t right_hip_roll;
    scalar_t right_hip_yaw;
    scalar_t right_knee_pitch;
    scalar_t right_ankle_pitch;
    scalar_t right_ankle_roll;
    // MOGA Waist
    scalar_t waist_yaw;
    scalar_t waist_pitch;
    // MOGA Left Arm
    scalar_t left_shoulder_pitch;
    scalar_t left_shoulder_roll;
    scalar_t left_shoulder_yaw;
    scalar_t left_elbow_pitch;
    scalar_t left_elbow_yaw;
    scalar_t left_wrist_pitch;
    scalar_t left_wrist_roll;
    // MOGA Right Arm
    scalar_t right_shoulder_pitch;
    scalar_t right_shoulder_roll;
    scalar_t right_shoulder_yaw;
    scalar_t right_elbow_pitch;
    scalar_t right_elbow_yaw;
    scalar_t right_wrist_pitch;
    scalar_t right_wrist_roll;
    // MOGA Head
    scalar_t head_yaw;
    scalar_t head_pitch;
  };

struct JoyInfo {
  float axes[8];
  int buttons[11];
};

struct Proprioception {
  vector_t jointPos;
  vector_t jointVel;
  vector3_t baseAngVel;
  vector3_t baseEulerXyz;
  vector_t refPos;
  vector_t posDiff;
  vector3_t projectedGravity;
};

struct Command {
  std::atomic<scalar_t> x;
  std::atomic<scalar_t> y;
  std::atomic<scalar_t> yaw;
};

// 单个电机的测试参数
struct TestMotorParams {
  int id;                    // 电机ID（关节索引从1开始）
  float kp;                  // PD控制器刚度
  float kd;                  // PD控制器阻尼
  float start_rad;           // 起始角度（弧度）
  float end_rad;             // 结束角度（弧度）
  float frequency;           // 摆动频率（Hz）
  std::string waveform_type; // 波形类型: sine, fourier, triangle, step, linear, chirp
  float step_time;           // 阶跃时间（仅step波形）
  float start_tau;           // 起始力矩（仅step波形）
  float end_tau;             // 结束力矩（仅step波形）
  
  // ---- 新增，仅供 chirp 用 ----
  float f0_hz   = 0.1f;      // 起始频率
  float f1_hz   = 10.0f;     // 结束频率(会被平台上限裁剪)
  float T_sec   = 30.0f;     // 扫频时长
  float phase   = 0.0f;      // 可选相位偏置
  float ramp_sec= 2.0f;      // 渐入/渐出时间
};


class RLControllerBase: public controller_interface::ControllerInterface {
public:
    enum class Mode : uint8_t {DEFAULT, LIE, STAND, WALK, TEST };
    enum class WalkMode : uint8_t {DEFAULT, SWING_ARM, LIFT_BOX, PUT_DOWN_BOX , WAVE_HAND, SHAKE_HAND };
    WalkMode walkmode_ = WalkMode::DEFAULT;
    WalkMode last_walkmode_ = WalkMode::DEFAULT;

    RLControllerBase();
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

    virtual bool loadModel() { return false; };
    virtual bool loadRLCfg() { return false; };
    virtual void computeActions(){};
    virtual void computeObservation(){};

    virtual void handleDefaultMode();
    virtual void handleLieMode();
    virtual void handleStandMode();
    virtual void handleTestMode();
    virtual void starting();

    // 从yaml文件加载测试模式参数
    virtual bool loadTestMotorParams();

    virtual void handleWalkMode(){};

    // Function to read CSV file and store data in a 2D vector
    std::vector<std::vector<double>> readCSV(const std::string& filePath);

    // 通用CSV加载函数
    std::vector<std::vector<double>> load_mocap_csv(const std::string& path,
                                                    const std::vector<size_t>& indices,
                                                    size_t stride = 1);

    virtual void calcFrameBlend(double phase, int trajectory_num_frames, size_t& frame_idx0, size_t& frame_idx1, double& blend);

    // 保留基本的轨迹相关变量
    double trajectory_duration;
    int trajectory_num_frames;
    std::vector<std::vector<double>> ref_dof_pos;
    std::vector<double> ref_dof_pos_low;
    std::vector<double> ref_dof_pos_high;
    std::vector<double> ref_dof_pos_cal;

protected:
    virtual void updateStateEstimation(const rclcpp::Time & time, const rclcpp::Duration & period);
    virtual void updateVelocityRegulator(const rclcpp::Time & time);

    virtual void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    virtual void joyInfoCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    // virtual void callbackImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    virtual void publishImuTransform(const sensor_msgs::msg::Imu &imu_msg);

    // 辅助函数：创建publisher
    template<typename T>
    auto create_pub(const std::string& topic, int qos = 1) {
        return get_node()->create_publisher<T>(topic, qos);
    }

    // 辅助函数：创建subscriber
    template<typename T>
    auto create_sub(const std::string& topic, void (RLControllerBase::*callback)(const typename T::SharedPtr), int qos = 1) {
        return get_node()->create_subscription<T>(topic, qos,
            std::bind(callback, this, std::placeholders::_1));
    }

    // 辅助函数：创建边缘触发的状态切换订阅器
    auto make_edge_triggered_sub(const std::string& topic, Mode old_mode, Mode new_mode,
                                const std::string& log_message, int qos = 1) {
        return get_node()->create_subscription<std_msgs::msg::Float32>(topic, qos,
            [this, old_mode, new_mode, log_message](const std_msgs::msg::Float32::SharedPtr /*msg*/) {
                rclcpp::Duration t(0, 500000000);
                auto currentTime = rclcpp::Clock(RCL_ROS_TIME).now();
                if (currentTime - switchTime > t) {
                    if (mode_ == old_mode) {
                        standPercent_ = 0;
                        for (size_t i = 0; i < hybridJointHandles_.size(); i++) {
                            currentJointAngles_[i] = hybridJointHandles_[i]->getPosCurr();
                        }
                        mode_ = new_mode;
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), log_message);
                    }
                    switchTime = rclcpp::Clock(RCL_ROS_TIME).now();
                }
            });
    }

    // 辅助函数：创建条件边缘触发的状态切换订阅器
    auto make_conditional_edge_triggered_sub(const std::string& topic, Mode old_mode, Mode new_mode,
                                           const std::string& log_message,
                                           std::function<bool()> condition, int qos = 1) {
        return get_node()->create_subscription<std_msgs::msg::Float32>(topic, qos,
            [this, old_mode, new_mode, log_message, condition](const std_msgs::msg::Float32::SharedPtr /*msg*/) {
                rclcpp::Duration t(0, 500000000);
                auto currentTime = rclcpp::Clock(RCL_ROS_TIME).now();
                if (currentTime - switchTime > t && condition()) {
                    if (mode_ == old_mode) {
                        standPercent_ = 0;
                        for (size_t i = 0; i < hybridJointHandles_.size(); i++) {
                            currentJointAngles_[i] = hybridJointHandles_[i]->getPosCurr();
                        }
                        mode_ = new_mode;
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), log_message);
                    }
                    switchTime = rclcpp::Clock(RCL_ROS_TIME).now();
                }
            });
    }

    // 辅助函数：执行状态切换的通用逻辑
    void perform_mode_switch(Mode new_mode, const std::string& log_message) {
        standPercent_ = 0;
        for (size_t i = 0; i < hybridJointHandles_.size(); i++) {
            currentJointAngles_[i] = hybridJointHandles_[i]->getPosCurr();
        }
        mode_ = new_mode;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), log_message);
    }

    // 辅助函数：检查时间间隔的通用逻辑
    bool check_time_interval() {
        rclcpp::Duration t(0, 500000000);
        auto currentTime = rclcpp::Clock(RCL_ROS_TIME).now();
        if (currentTime - switchTime > t) {
            switchTime = rclcpp::Clock(RCL_ROS_TIME).now();
            return true;
        }
        return false;
    }

    // cmd_vel 超时减速相关变量
    double cmd_vel_timeout_duration_;      // cmd_vel 超时阈值（秒）
    double cmd_vel_deceleration_rate_;     // 速度衰减率（米/秒^2）
    rclcpp::Time last_cmd_vel_time_;       // 最后接收 cmd_vel 的时间
    geometry_msgs::msg::Twist internal_command_; // 内部速度命令，用于减速控制

    // 速度调节器相关变量
    double max_linear_acceleration_;       // 最大线性加速度（米/秒^2）
    double max_angular_acceleration_;      // 最大角加速度（弧度/秒^2）
    double velocity_update_rate_;          // 速度更新频率（Hz）
    geometry_msgs::msg::Twist target_command_;   // 目标速度命令
    geometry_msgs::msg::Twist current_command_;  // 当前平滑后的速度命令
    rclcpp::Time last_velocity_update_time_;     // 最后速度更新时间

    std::vector<std::string> joint_names_;
    std::vector<std::string> command_interface_types_;
    std::vector<std::string> state_interface_types_;

    // IMU 相关成员变量
    std::string imu_name_;
    std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;

    bool new_msg_ = false;
    rclcpp::Time start_time_;


    /* sim !!!*/
    std::vector<std::unique_ptr<HybridJointHandle>> hybridJointHandles_;
    bool use_sim_handles_;

    Mode mode_;
    int64_t loopCount_;
    Command command_;
    RLRobotCfg robotCfg_{};

    JointState standjointState_{
                              // MOGA Left Leg: hip_pitch, hip_roll, hip_yaw, knee_pitch, ankle_pitch, ankle_roll
                              -0.0, 0.0, -0.0, 0.0, -0.0, 0.0,
                              // MOGA Right Leg: hip_pitch, hip_roll, hip_yaw, knee_pitch, ankle_pitch, ankle_roll
                              -0.0, 0.0, -0.0, 0.0, -0.0, 0.0,
                              // MOGA Waist: waist_yaw, waist_pitch
                              0.0, 0.0,
                              // MOGA Left Arm: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_yaw, wrist_pitch, wrist_roll
                              0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
                              // MOGA Right Arm: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_yaw, wrist_pitch, wrist_roll
                              0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              // MOGA Head: head_yaw, head_pitch
                              0.0, 0.0
                              };

    JointState liejointState_{
                              // MOGA Left Leg: hip_pitch, hip_roll, hip_yaw, knee_pitch, ankle_pitch, ankle_roll
                              -0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
                              // MOGA Right Leg: hip_pitch, hip_roll, hip_yaw, knee_pitch, ankle_pitch, ankle_roll
                              -0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
                              // MOGA Waist: waist_yaw, waist_pitch
                              0.0, 0.0,
                              // MOGA Left Arm: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_yaw, wrist_pitch, wrist_roll
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              // MOGA Right Arm: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_yaw, wrist_pitch, wrist_roll
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              // MOGA Head: head_yaw, head_pitch
                              0.0, 0.0
                                };
    JoyInfo joyInfo;
    std::atomic_bool emergency_stop{false};
    std::atomic_bool start_control{false};
    rclcpp::Time switchTime;

    vector_t rbdState_;
    vector_t measuredRbdState_;
    Proprioception propri_;
    sensor_msgs::msg::Imu imu_data_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joyInfoSub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr emgStopSub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr startCtrlSub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr walkModeSub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr switchModeSub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr positionCtrlSub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr realJointPosPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr realJointVelPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr realJointTorquePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rlPlannedJointPosPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rlPlannedJointVelPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rlPlannedTorquePublisher_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr realImuAngularVelPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr realImuLinearAccPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr realImuEulerXyzPulbisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr phasePublisher_;

    // 控制周期分析相关变量
    std_msgs::msg::Float64 updateTimeMsg_;
    std_msgs::msg::Float64 updatePeriodMsg_;
    std_msgs::msg::Float64 calUpdateTimeMsg_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr updateTimePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr updatePeriodPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr calUpdateTimePublisher_;

    int actuatedDofNum_;
    vector_t currentPos_;
    vector_t currentVel_;
    vector_t Posdes_;
    double phase_ = 0.;
    double current_time_;
    double cmd_norm_;

    double lie_kp_large_, lie_kd_large_, lie_kp_ankle_, lie_kd_ankle_,
          stance_kp_large_, stance_kd_large_, stance_kp_ankle_, stance_kd_ankle_;

    double fps = 113.0;
    int frame_idx0;
    int frame_idx1;
    double blend;
    vector_t allJointPos_;
    int actionsSize_;

    std::vector<int> leg_joint_mapping = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

    // 基本映射
    std::vector<size_t> non_zero_indices = {0, 7}; // mocap数据中提取参考轨迹.
    std::vector<int> swing_arm_mapping = {13,20};
    std::vector<int> joint_mapping_fixed = {12, 14, 15, 16,
                                            17, 18, 19, 21, 22, 23,
                                            24, 25, 26};

private:
    // PD stand
    std::vector<scalar_t> currentJointAngles_;
    std::vector<scalar_t> initJointAngles_;
    vector_t standJointAngles_;
    vector_t lieJointAngles_;
    vector_t sitJointAngles_;

    scalar_t standPercent_;
    scalar_t standDuration_;

    // Test mode related variables
    std::vector<TestMotorParams> test_motor_params_;  // 每个电机的独立参数
    bool test_params_initialized_ = false;
    std::chrono::high_resolution_clock::time_point test_start_time_;
    
    // Test mode data recording variables
    static constexpr int MAX_TEST_COUNT = 1000;  // 单次测试最多记录的数据条数
    int test_count_ = 0;  // 当前测试计数
    int test_decimation_ = 10;  // 数据采样率（每N次调用记录一次）
    std::string test_csv_save_path_;  // CSV文件保存路径
    
    // 数据记录容器（按电机ID索引）
    std::map<int, std::vector<double>> test_time_values_;        // 时间戳
    std::map<int, std::vector<double>> test_pos_des_values_;      // 目标位置
    std::map<int, std::vector<double>> test_pos_current_values_;  // 当前位置
    std::map<int, std::vector<double>> test_vel_current_values_;  // 当前速度
    std::map<int, std::vector<double>> test_torque_planned_values_;  // 计划扭矩
    std::map<int, std::vector<double>> test_torque_real_values_;     // 实际扭矩
    std::map<int, std::vector<double>> test_kp_values_;             // kp参数
    std::map<int, std::vector<double>> test_kd_values_;             // kd参数

    // 延迟控制相关变量
    static constexpr size_t LAGGED_BUFFER_SIZE = 1000;        // 缓冲区大小
    static constexpr size_t LAGGED_POS_DES_DELAY_STEPS = 8;   // PosDes 延迟步数
    static constexpr size_t LAGGED_POS_CURR_DELAY_STEPS = 0;  // PosCurr 延迟步数

    std::vector<std::vector<double>> lagged_pos_des_buffer_;   // 历史 PosDes 缓冲区 [joint_num][buffer_size]
    std::vector<std::vector<double>> lagged_pos_curr_buffer_;  // 历史 PosCurr 缓冲区 [joint_num][buffer_size]
    size_t lagged_buffer_index_;  // 当前缓冲区写入位置（两个缓冲区共用同一个索引）

};


}

