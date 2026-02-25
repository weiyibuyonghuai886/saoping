#include <rl_controllers/RLControllerBase.h>
#include "rclcpp/parameter_client.hpp"
#include <rl_controllers/RotationTools.h>
#include <rl_controllers/utilities.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <random>
#include <cmath>
#include <filesystem>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

namespace legged
{

  using config_type = controller_interface::interface_configuration_type;

  // ========== Chirp 扫频辅助函数 ==========
  inline double chirp_phase_lin(double t, double f0, double f1, double T){
    // 线性扫频相位：phi(t)=2π*(f0*t + (f1-f0)*t^2/(2T))
    double k = (f1 - f0) / T;
    return 2.0 * M_PI * (f0 * t + 0.5 * k * t * t);
  }

  inline double envelope_lin(double t, double T, double ramp){
    if (ramp <= 0.0) return 1.0;
    if (t < ramp)                 return t / ramp;               // 渐入
    if (t > T - ramp)             return std::max(0.0, (T - t) / ramp); // 渐出
    return 1.0;
  }
  // ========== End ==========

  RLControllerBase::RLControllerBase() : controller_interface::ControllerInterface() {
    // 初始化 IMUSensor（将在on_init中设置名称）
    imu_sensor_ = nullptr;

    // 初始化延迟控制变量
    lagged_buffer_index_ = 0;
  }

  // First
  controller_interface::CallbackReturn RLControllerBase::on_init()
  {
    // 初始化更新周期消息
    updateTimeMsg_.data = 0.0;
    updatePeriodMsg_.data = 0.0;
    calUpdateTimeMsg_.data = 0.0;

    // 将参数服务器中的变量存入joint_names_,command_interface, state_interface
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
        auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
        auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    actuatedDofNum_ = joint_names_.size();

    // // 初始化 IMU 相关参数
    imu_name_ = auto_declare<std::string>("imu_name", "imu_sensor");
    // 创建 IMUSensor 实例
    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(imu_name_);

    // 速度调节器参数初始化
    max_linear_acceleration_ = auto_declare<double>("LeggedRobotCfg.commands.max_linear_acceleration", 2.0);
    max_angular_acceleration_ = auto_declare<double>("LeggedRobotCfg.commands.max_angular_acceleration", 3.0);
    velocity_update_rate_ = auto_declare<double>("LeggedRobotCfg.commands.update_rate", 500.0); // Hz

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IMU configuration initialized:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  IMU name: %s", imu_name_.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Velocity regulator parameters:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Max linear acceleration: %.2f m/s²", max_linear_acceleration_);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Max angular acceleration: %.2f rad/s²", max_angular_acceleration_);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Update rate: %.1f Hz", velocity_update_rate_);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());

    // Load policy model and rl cfg
    if (!loadModel())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "[RLControllerBase] Failed to load the model. Ensure the path is correct and accessible.");
      return CallbackReturn::FAILURE;
    }
    if (!loadRLCfg())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "[RLControllerBase] Failed to load the rl config. Ensure the yaml is correct and accessible.");
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "load arm motion retargeted data");
    std::string packageSharePath = ament_index_cpp::get_package_share_directory("rl_controllers");
    std::string csvFilePath = packageSharePath + "/pkl_data/interpolated_joint_angles_new_phase_bias.csv";
    ref_dof_pos = load_mocap_csv(csvFilePath, non_zero_indices);
    trajectory_num_frames = ref_dof_pos.size();
    trajectory_duration = (trajectory_num_frames > 1) ? (trajectory_num_frames - 1) / fps : 0.0;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "trajectory_duration: " << trajectory_duration);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "trajectory_num_frames: " << trajectory_num_frames);

    // 简化矢量初始化
    if (!ref_dof_pos.empty()) {
      ref_dof_pos_cal = std::vector<double>(ref_dof_pos[0].size(), 0.0);
    }

    switchTime = rclcpp::Clock(RCL_ROS_TIME).now();
    standJointAngles_ = Eigen::VectorXd::Zero(actuatedDofNum_);
    lieJointAngles_ = Eigen::VectorXd::Zero(actuatedDofNum_);
    allJointPos_ = Eigen::VectorXd::Zero(actuatedDofNum_);

    auto &StandState = standjointState_;
    auto &LieState = liejointState_;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "actuatedDofNum_: " << actuatedDofNum_);

    // MOGA joint configuration
    lieJointAngles_ <<  LieState.left_hip_pitch, LieState.left_hip_roll, LieState.left_hip_yaw,
                        LieState.left_knee_pitch, LieState.left_ankle_pitch, LieState.left_ankle_roll,
                        LieState.right_hip_pitch, LieState.right_hip_roll, LieState.right_hip_yaw,
                        LieState.right_knee_pitch, LieState.right_ankle_pitch, LieState.right_ankle_roll,
                        LieState.waist_yaw, LieState.waist_pitch,
                        LieState.left_shoulder_pitch, LieState.left_shoulder_roll, LieState.left_shoulder_yaw,
                        LieState.left_elbow_pitch, LieState.left_elbow_yaw, LieState.left_wrist_pitch, LieState.left_wrist_roll,
                        LieState.right_shoulder_pitch, LieState.right_shoulder_roll, LieState.right_shoulder_yaw,
                        LieState.right_elbow_pitch, LieState.right_elbow_yaw, LieState.right_wrist_pitch, LieState.right_wrist_roll,
                        LieState.head_yaw, LieState.head_pitch;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "lieJointAngles_: " << lieJointAngles_);

    // MOGA joint configuration
    standJointAngles_ << StandState.left_hip_pitch, StandState.left_hip_roll, StandState.left_hip_yaw,
                         StandState.left_knee_pitch, StandState.left_ankle_pitch, StandState.left_ankle_roll,
                         StandState.right_hip_pitch, StandState.right_hip_roll, StandState.right_hip_yaw,
                         StandState.right_knee_pitch, StandState.right_ankle_pitch, StandState.right_ankle_roll,
                         StandState.waist_yaw, StandState.waist_pitch,
                         StandState.left_shoulder_pitch, StandState.left_shoulder_roll, StandState.left_shoulder_yaw,
                         StandState.left_elbow_pitch, StandState.left_elbow_yaw, StandState.left_wrist_pitch, StandState.left_wrist_roll,
                         StandState.right_shoulder_pitch, StandState.right_shoulder_roll, StandState.right_shoulder_yaw,
                         StandState.right_elbow_pitch, StandState.right_elbow_yaw, StandState.right_wrist_pitch, StandState.right_wrist_roll,
                         StandState.head_yaw, StandState.head_pitch;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "standJointAngles_: " << standJointAngles_);

    // 使用表驱动方式创建publishers - 简化版本
    auto create_publishers = [this]() {
        // Float64MultiArray publishers
        std::vector<std::pair<std::string, std::function<void(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>)>>> float64_multi_publishers = {
            {"data_analysis/real_joint_pos", [this](auto pub) { realJointPosPublisher_ = pub; }},
            {"data_analysis/real_joint_vel", [this](auto pub) { realJointVelPublisher_ = pub; }},
            {"data_analysis/real_joint_torque", [this](auto pub) { realJointTorquePublisher_ = pub; }},
            {"data_analysis/rl_planned_joint_pos", [this](auto pub) { rlPlannedJointPosPublisher_ = pub; }},
            {"data_analysis/rl_planned_joint_vel", [this](auto pub) { rlPlannedJointVelPublisher_ = pub; }},
            {"data_analysis/rl_planned_joint_torque", [this](auto pub) { rlPlannedTorquePublisher_ = pub; }},
            {"data_analysis/imu_angular_vel", [this](auto pub) { realImuAngularVelPublisher_ = pub; }},
            {"data_analysis/imu_linear_acc", [this](auto pub) { realImuLinearAccPublisher_ = pub; }},
            {"data_analysis/imu_euler_xyz", [this](auto pub) { realImuEulerXyzPulbisher_ = pub; }}
        };

        // 批量创建Float64MultiArray publishers
        for (const auto& [topic, assign_func] : float64_multi_publishers) {
            assign_func(create_pub<std_msgs::msg::Float64MultiArray>(topic));
        }

        // Float64 publisher
        phasePublisher_ = create_pub<std_msgs::msg::Float64>("data_analysis/phase");
        updateTimePublisher_ = create_pub<std_msgs::msg::Float64>("data_analysis/update_time");
        updatePeriodPublisher_ = create_pub<std_msgs::msg::Float64>("data_analysis/update_period");
        calUpdateTimePublisher_ = create_pub<std_msgs::msg::Float64>("data_analysis/cal_update_time");
    };

    create_publishers();

    // 创建subscribers - 使用辅助函数消除重复lambda
    cmdVelSub_ = create_sub<geometry_msgs::msg::Twist>("/cmd_vel", &RLControllerBase::cmdVelCallback);
    joyInfoSub_ = create_sub<sensor_msgs::msg::Joy>("/joy", &RLControllerBase::joyInfoCallback);

    // 创建紧急停止订阅器（简单回调）
    emgStopSub_ = get_node()->create_subscription<std_msgs::msg::Float32>("/emergency_stop", 1,
        [this](const std_msgs::msg::Float32::SharedPtr /*msg*/) {
            emergency_stop = true;
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Emergency Stop");
        });

    // 创建start_control订阅器（特殊逻辑）
    startCtrlSub_ = get_node()->create_subscription<std_msgs::msg::Float32>("/start_control", 1,
        [this](const std_msgs::msg::Float32::SharedPtr /*msg*/) {
            if (check_time_interval()) {
                if (!start_control) {
                    start_control = true;
                    perform_mode_switch(Mode::LIE, "Start Control");
                } else {
                    start_control = false;
                    perform_mode_switch(Mode::DEFAULT, "ShutDown Control");
                }
            }
        });

    // 使用辅助函数创建边缘触发的状态切换订阅器
    switchModeSub_ = get_node()->create_subscription<std_msgs::msg::Float32>("/switch_mode", 1,
        [this](const std_msgs::msg::Float32::SharedPtr /*msg*/) {
            if (check_time_interval()) {
                if (start_control == true) {
                    if (mode_ == Mode::STAND) {
                        perform_mode_switch(Mode::LIE, "STAND2LIE");
                    } else if (mode_ == Mode::LIE) {
                        perform_mode_switch(Mode::STAND, "LIE2STAND");
                    }
                } else {
                    // 当start_control为false时，在DEFAULT和TEST模式之间切换
                    if (mode_ == Mode::DEFAULT) {
                        // 从yaml文件重新加载测试参数
                        if (loadTestMotorParams()) {
                            mode_ = Mode::TEST;
                            test_params_initialized_ = false;
                            test_start_time_ = std::chrono::high_resolution_clock::now();
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DEFAULT2TEST - Parameters loaded from YAML");
                        } else {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load test parameters, staying in DEFAULT mode");
                        }
                    } else if (mode_ == Mode::TEST) {
                        mode_ = Mode::DEFAULT;
                        test_params_initialized_ = false;
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TEST2DEFAULT");
                    }
                }
            }
        });

    // 使用辅助函数创建其他边缘触发的订阅器
    walkModeSub_ = get_node()->create_subscription<std_msgs::msg::Float32>("/walk_mode", 1,
        [this](const std_msgs::msg::Float32::SharedPtr /*msg*/) {
            if (check_time_interval() && mode_ == Mode::STAND) {
                loopCount_ = 0;
                perform_mode_switch(Mode::WALK, "STAND2WALK");
            }
        });

    // 创建position_control订阅器（多条件切换）
    positionCtrlSub_ = get_node()->create_subscription<std_msgs::msg::Float32>("/position_control", 1,
        [this](const std_msgs::msg::Float32::SharedPtr /*msg*/) {
            if (check_time_interval()) {
                if (mode_ == Mode::WALK) {
                    perform_mode_switch(Mode::STAND, "WALK2STAND");
                } else if (mode_ == Mode::DEFAULT) {
                    perform_mode_switch(Mode::LIE, "DEF2LIE");
                }
            }
        });

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "rl_controller initialized!!!!!!!!!");

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn RLControllerBase::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state; // 消除未使用参数警告

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rlccpp"), "use_sim_handles_ : " << use_sim_handles_);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  // Second
  controller_interface::InterfaceConfiguration RLControllerBase::command_interface_configuration()
      const
  {
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    // 使用 lambda 函数简化关节遍历和接口生成
    auto add_joint_interfaces = [&conf, this](const std::string& joint_name) {
      for (const auto& interface_type : command_interface_types_) {
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    };

    std::for_each(joint_names_.begin(), joint_names_.end(), add_joint_interfaces);

    return conf;
  }

  controller_interface::InterfaceConfiguration RLControllerBase::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    // 使用 lambda 函数简化关节状态接口生成
    auto add_joint_state_interfaces = [&conf, this](const std::string& joint_name) {
      for (const auto& interface_type : state_interface_types_) {
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    };

    std::for_each(joint_names_.begin(), joint_names_.end(), add_joint_state_interfaces);

    // 添加 IMU 状态接口
    if (imu_sensor_) {
      // 使用 IMUSensor 获取接口名称
      auto imu_interface_names = imu_sensor_->get_state_interface_names();
      conf.names.insert(conf.names.end(), imu_interface_names.begin(), imu_interface_names.end());
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using IMUSensor for IMU state interfaces (found %zu interfaces)", imu_interface_names.size());
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Total state interfaces configured: %zu (joints: %zu, IMU: %zu)",
                conf.names.size(), joint_names_.size() * state_interface_types_.size(),
                imu_sensor_ ? imu_sensor_->get_state_interface_names().size() : 0);

    return conf;
  }

  controller_interface::CallbackReturn RLControllerBase::on_activate(const rclcpp_lifecycle::State &)
  {
    // sim !!!
    std::map<std::string, std::vector<int>> state_interface_indices;
    for (int i = 0; i < static_cast<int>(state_interfaces_.size()); i++)
    {
      state_interface_indices[state_interfaces_[i].get_prefix_name()].push_back(i);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", state_interfaces_[i].get_name().c_str());
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "size: " << state_interfaces_.size());

    std::map<std::string, std::vector<int>> command_interface_indices;
    for (int i = 0; i < static_cast<int>(command_interfaces_.size()); i++)
    {
      command_interface_indices[command_interfaces_[i].get_prefix_name()].push_back(i);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", command_interfaces_[i].get_name().c_str());
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "command success");

    // 使用简洁的lambda表达式构建关节句柄
    auto build_handle = [&](const std::string& name, bool sim) -> std::unique_ptr<HybridJointHandle> {
        auto s = state_interface_indices[name][0];
        auto c = command_interface_indices[name][0];

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint %s: state_idx=%d, command_idx=%d, use_sim=%s",
                    name.c_str(), s, c, sim ? "true" : "false");

        if (sim) {
            return std::make_unique<HybridJointHandleSim>(
                std::ref(state_interfaces_[s]),
                std::ref(state_interfaces_[s + 1]),
                std::ref(state_interfaces_[s + 2]),
                std::ref(command_interfaces_[c]));
        } else {
            return std::make_unique<HybridJointHandleHW>(
                std::ref(state_interfaces_[s]),
                std::ref(state_interfaces_[s + 1]),
                std::ref(state_interfaces_[s + 2]),
                std::ref(command_interfaces_[c]),
                std::ref(command_interfaces_[c + 1]),
                std::ref(command_interfaces_[c + 2]),
                std::ref(command_interfaces_[c + 3]),
                std::ref(command_interfaces_[c + 4]));
        }
    };

    for (auto& j : joint_names_) {
        hybridJointHandles_.push_back(build_handle(j, use_sim_handles_));
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully built %zu hybrid joint handles", joint_names_.size());

    // 检查 IMU 状态接口是否可用
    if (imu_sensor_) {
      // 使用 IMUSensor 分配状态接口
      bool success = imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
      if (success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully assigned IMU state interfaces using IMUSensor");
      } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to assign IMU state interfaces using IMUSensor, falling back to manual method");
      }
    }

    starting();

    return CallbackReturn::SUCCESS;
  }

  void RLControllerBase::starting()
  {
    // ⚠️ 必须先初始化延迟控制缓冲区，再调用 updateStateEstimation
    lagged_pos_des_buffer_.resize(hybridJointHandles_.size());
    lagged_pos_curr_buffer_.resize(hybridJointHandles_.size());
    for (size_t i = 0; i < hybridJointHandles_.size(); i++) {
      lagged_pos_des_buffer_[i].resize(LAGGED_BUFFER_SIZE, 0.0);
      lagged_pos_curr_buffer_[i].resize(LAGGED_BUFFER_SIZE, 0.0);
    }
    lagged_buffer_index_ = 0;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized lagged position buffers: %zu joints x %zu steps",
                hybridJointHandles_.size(), LAGGED_BUFFER_SIZE);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  - PosDes delay: %zu steps, PosCurr delay: %zu steps",
                LAGGED_POS_DES_DELAY_STEPS, LAGGED_POS_CURR_DELAY_STEPS);

    // 更新状态
    updateStateEstimation(get_node()->now(), rclcpp::Duration::from_seconds(0.002));
    currentJointAngles_ = std::vector<scalar_t>(actuatedDofNum_, 0.0);

    // sim
    for (size_t i = 0; i < hybridJointHandles_.size(); i++)
    {
      currentJointAngles_[i] = hybridJointHandles_[i]->getPosCurr();
    }

    scalar_t durationSecs = 4.0;
    standDuration_ = durationSecs * 1000.0;
    standPercent_ = 0;
    mode_ = Mode::DEFAULT;
    loopCount_ = 0;

    // 初始化 cmd_vel 超时减速相关变量
    last_cmd_vel_time_ = get_node()->now();
    internal_command_.linear.x = 0.0;
    internal_command_.linear.y = 0.0;
    internal_command_.linear.z = 0.0;
    internal_command_.angular.x = 0.0;
    internal_command_.angular.y = 0.0;
    internal_command_.angular.z = 0.0;

    // 初始化速度调节器变量
    target_command_.linear.x = 0.0;
    target_command_.linear.y = 0.0;
    target_command_.angular.z = 0.0;
    current_command_.linear.x = 0.0;
    current_command_.linear.y = 0.0;
    current_command_.angular.z = 0.0;
    last_velocity_update_time_ = get_node()->now();
  }

  controller_interface::return_type RLControllerBase::update(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    updateTimeMsg_.data = time.seconds();
    updateTimePublisher_->publish(updateTimeMsg_);

    updatePeriodMsg_.data = period.seconds();
    updatePeriodPublisher_->publish(updatePeriodMsg_);

    std::chrono::high_resolution_clock::time_point update_start_time = std::chrono::high_resolution_clock::now();

    // // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "123");
    updateStateEstimation(time, period);

    // 更新速度调节器
    updateVelocityRegulator(time);

    // std::cout << " loopCount_" << loopCount_ << " standPercent_" << standPercent_ << " initJointAngles_[0]" << initJointAngles_[0]
    //         << "\n";

    switch (mode_)
    {
    case Mode::DEFAULT:
      handleDefaultMode();
      break;
    case Mode::LIE:
      handleLieMode();
      break;
    case Mode::STAND:
      handleStandMode();
      break;
    case Mode::WALK:
      handleWalkMode();
      break;
    case Mode::TEST:
      handleTestMode();
      break;
    default:
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Unexpected mode encountered: " << static_cast<int>(mode_));
      break;
    }

    if (emergency_stop)
    {
      for (size_t j = 0; j < hybridJointHandles_.size(); ++j)
      {
        hybridJointHandles_[j]->setCommand(0, 0, 0, 3, 0);
      }
    }

    if (emergency_stop && start_control)
    {
      emergency_stop = false;
      starting();
      start_control = false;
    }

    loopCount_++;

    std::chrono::high_resolution_clock::time_point update_end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> update_duration = std::chrono::duration_cast<std::chrono::duration<double>>(update_end_time - update_start_time);
    calUpdateTimeMsg_.data = update_duration.count();
    calUpdateTimePublisher_->publish(calUpdateTimeMsg_);

    return controller_interface::return_type::OK;
  }

  void RLControllerBase::handleDefaultMode()
  {
    for (size_t j = 0; j < hybridJointHandles_.size(); j++)
      {hybridJointHandles_[j]->setCommand(0, 0, 0.0, 1.0, 0);}
  }

  void RLControllerBase::handleLieMode()
  {
    if (standPercent_ <= 1)
    {
      for (size_t j = 0; j < hybridJointHandles_.size(); j++)
      {
        if (j == 4 || j == 5 || j == 10 || j == 11)
        {
          scalar_t pos_des = currentJointAngles_[j] * (1 - standPercent_) + lieJointAngles_[j] * standPercent_;
          hybridJointHandles_[j]->setCommand(pos_des, 0, lie_kp_ankle_, lie_kd_ankle_, 0);
        }
        else
        {
          scalar_t pos_des = currentJointAngles_[j] * (1 - standPercent_) + lieJointAngles_[j] * standPercent_;
          hybridJointHandles_[j]->setCommand(pos_des, 0, lie_kp_large_, lie_kd_large_, 0);
        }
      }
      standPercent_ += 1 / standDuration_;
      standPercent_ = std::min(standPercent_, scalar_t(1));
    }
  }

  void RLControllerBase::handleStandMode()
  {
    if (standPercent_ <= 1)
    {
      for (size_t j = 0; j < hybridJointHandles_.size(); j++)
      {
        if (j == 4 || j == 5 || j == 10 || j == 11)
        {
          scalar_t pos_des = currentJointAngles_[j] * (1 - standPercent_) + standJointAngles_[j] * standPercent_;
          hybridJointHandles_[j]->setCommand(pos_des, 0, stance_kp_ankle_, stance_kd_ankle_, 0);
        }
        else
        {
          scalar_t pos_des = currentJointAngles_[j] * (1 - standPercent_) + standJointAngles_[j] * standPercent_;
          hybridJointHandles_[j]->setCommand(pos_des, 0, stance_kp_large_, stance_kd_large_, 0);
        }
      }
      standPercent_ += 1 / standDuration_;
      standPercent_ = std::min(standPercent_, scalar_t(1));
    }

  }


  controller_interface::CallbackReturn RLControllerBase::on_deactivate(const rclcpp_lifecycle::State &)
  {
    // 释放 IMUSensor 接口
    if (imu_sensor_) {
      imu_sensor_->release_interfaces();
    }

    release_interfaces();

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn RLControllerBase::on_cleanup(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn RLControllerBase::on_error(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn RLControllerBase::on_shutdown(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  void RLControllerBase::updateStateEstimation(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    (void)time;    // 消除未使用参数警告
    (void)period;  // 消除未使用参数警告

    vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size()),
             jointTor(hybridJointHandles_.size()), output_torque(hybridJointHandles_.size()),
             pos_des_output(hybridJointHandles_.size()), vel_des_output(hybridJointHandles_.size());

    vector_t imuEulerXyz(3);
    quaternion_t quat;
    vector3_t angularVel, linearAccel;
    matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

    for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
    {
      jointPos(i) = hybridJointHandles_[i]->getPosCurr();
      allJointPos_(i) = hybridJointHandles_[i]->getPosCurr();
      jointVel(i) = hybridJointHandles_[i]->getVelCurr();
      jointTor(i) = hybridJointHandles_[i]->getTauCurr();

      pos_des_output(i) = hybridJointHandles_[i]->getPosDes();
      vel_des_output(i) = hybridJointHandles_[i]->getVelDes();

      // 存储当前的 PosDes 和 PosCurr 到缓冲区
      lagged_pos_des_buffer_[i][lagged_buffer_index_] = hybridJointHandles_[i]->getPosDes();
      lagged_pos_curr_buffer_[i][lagged_buffer_index_] = hybridJointHandles_[i]->getPosCurr();

      // 计算 PosDes 延迟索引
      size_t lagged_pos_des_index = (lagged_buffer_index_ + LAGGED_BUFFER_SIZE - LAGGED_POS_DES_DELAY_STEPS) % LAGGED_BUFFER_SIZE;
      double lagged_pos_des = lagged_pos_des_buffer_[i][lagged_pos_des_index];

      // 计算 PosCurr 延迟索引
      size_t lagged_pos_curr_index = (lagged_buffer_index_ + LAGGED_BUFFER_SIZE - LAGGED_POS_CURR_DELAY_STEPS) % LAGGED_BUFFER_SIZE;
      double lagged_pos_curr = lagged_pos_curr_buffer_[i][lagged_pos_curr_index];

      // 使用延迟的 PosDes 和 PosCurr 计算 output_torque
      output_torque(i) = hybridJointHandles_[i]->getFeedforward() +
                         hybridJointHandles_[i]->getKp() * (lagged_pos_des - lagged_pos_curr) +
                         hybridJointHandles_[i]->getKd() * (hybridJointHandles_[i]->getVelDes() - hybridJointHandles_[i]->getVelCurr());
    }

    // 更新缓冲区索引（循环）
    lagged_buffer_index_ = (lagged_buffer_index_ + 1) % LAGGED_BUFFER_SIZE;


    // 如果状态接口中有 IMU 数据，优先使用状态接口的数据
    if (imu_sensor_) {
      // 使用 IMUSensor 读取数据
      try {
        imu_sensor_->get_values_as_message(imu_data_);

        // 更新四元数
        quat.x() = imu_data_.orientation.x;
        quat.y() = imu_data_.orientation.y;
        quat.z() = imu_data_.orientation.z;
        quat.w() = imu_data_.orientation.w;

        // 更新角速度
        angularVel(0) = imu_data_.angular_velocity.x;
        angularVel(1) = imu_data_.angular_velocity.y;
        angularVel(2) = imu_data_.angular_velocity.z;

        // 更新线性加速度
        linearAccel(0) = imu_data_.linear_acceleration.x;
        linearAccel(1) = imu_data_.linear_acceleration.y;
        linearAccel(2) = imu_data_.linear_acceleration.z;

      } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to read IMU data from IMUSensor: %s", e.what());
      }
    }
    publishImuTransform(imu_data_);

    // vector_t selectedJointPos(12);
    // for (size_t i = 0; i < 12; ++i) {
    // selectedJointPos(i) = jointPos(i); // Indices 1-12
    // }

    // vector_t selectedJointVel(12);
    // for (size_t i = 0; i < 12; ++i) {
    // selectedJointVel(i) = jointVel(i); // Indices 1-12
    // }
    propri_.jointPos = jointPos;
    propri_.jointVel = jointVel;
    propri_.baseAngVel = angularVel;

    vector3_t gravityVector(0, 0, -1);
    vector3_t zyx = quatToZyx(quat);
    matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();
    propri_.projectedGravity = inverseRot * gravityVector;
    propri_.baseEulerXyz = quatToXyz(quat);
    // propri_.baseEulerXyz[1] -= 0.05;
    // propri_.baseEulerXyz[0] += 0.012;
    // propri_.baseEulerXyz[1] += 0.02;
    // phase_ = time.seconds();

    for (size_t i = 0; i < 3; ++i)
    {
      imuEulerXyz(i) = propri_.baseEulerXyz[i];
    }

    realJointPosPublisher_->publish(createFloat64MultiArrayFromVector(jointPos));
    realJointVelPublisher_->publish(createFloat64MultiArrayFromVector(jointVel));
    realJointTorquePublisher_->publish(createFloat64MultiArrayFromVector(jointTor));

    realImuAngularVelPublisher_->publish(createFloat64MultiArrayFromVector(angularVel));
    realImuLinearAccPublisher_->publish(createFloat64MultiArrayFromVector(linearAccel));
    realImuEulerXyzPulbisher_->publish(createFloat64MultiArrayFromVector(imuEulerXyz));

    rlPlannedJointPosPublisher_->publish(createFloat64MultiArrayFromVector(pos_des_output));
    rlPlannedJointVelPublisher_->publish(createFloat64MultiArrayFromVector(vel_des_output));
    rlPlannedTorquePublisher_->publish(createFloat64MultiArrayFromVector(output_torque));


  }

  void RLControllerBase::updateVelocityRegulator(const rclcpp::Time &time)
  {
    // 计算时间间隔
    double dt = (time - last_velocity_update_time_).seconds();
    last_velocity_update_time_ = time;

    // 如果时间间隔太大或太小，跳过本次更新
    if (dt <= 0.0 || dt > 0.1) {
      return;
    }

    // 计算每个轴的最大速度变化量
    double max_linear_change = max_linear_acceleration_ * dt;
    double max_angular_change = max_angular_acceleration_ * dt;

    // 限制线性速度变化
    double linear_x_diff = target_command_.linear.x - current_command_.linear.x;
    if (std::abs(linear_x_diff) > max_linear_change) {
      current_command_.linear.x += (linear_x_diff > 0 ? max_linear_change : -max_linear_change);
    } else {
      current_command_.linear.x = target_command_.linear.x;
    }

    double linear_y_diff = target_command_.linear.y - current_command_.linear.y;
    if (std::abs(linear_y_diff) > max_linear_change) {
      current_command_.linear.y += (linear_y_diff > 0 ? max_linear_change : -max_linear_change);
    } else {
      current_command_.linear.y = target_command_.linear.y;
    }

    // 限制角速度变化
    double angular_z_diff = target_command_.angular.z - current_command_.angular.z;
    if (std::abs(angular_z_diff) > max_angular_change) {
      current_command_.angular.z += (angular_z_diff > 0 ? max_angular_change : -max_angular_change);
    } else {
      current_command_.angular.z = target_command_.angular.z;
    }

    // 更新command_结构体（使用调节后的平滑速度）
    command_.x = current_command_.linear.x;
    command_.y = current_command_.linear.y;
    command_.yaw = current_command_.angular.z;

    // 更新internal_command_（使用调节后的平滑速度）
    internal_command_.linear.x = current_command_.linear.x;
    internal_command_.linear.y = current_command_.linear.y;
    internal_command_.angular.z = current_command_.angular.z;
  }

  void RLControllerBase::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 设置目标速度而不是直接设置当前速度
    target_command_.linear.x = msg->linear.x;
    target_command_.linear.y = msg->linear.y;
    target_command_.angular.z = msg->angular.z;

    // 记录最后接收 cmd_vel 的时间
    last_cmd_vel_time_ = get_node()->now();

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
        "Received target velocity: linear_x=%.3f, linear_y=%.3f, angular_z=%.3f",
        target_command_.linear.x, target_command_.linear.y, target_command_.angular.z);
  }

  void RLControllerBase::joyInfoCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->header.frame_id.empty())
    {
      return;
    }
    // memcpy(joyInfo.axes, msg.axes, sizeof(joyInfo.axes));
    // memcpy(joyInfo.buttons, msg.buttons, sizeof(joyInfo.buttons));
    for (int i = 0; i < static_cast<int>(msg->axes.size()); i++)
    {
      joyInfo.axes[i] = msg->axes[i];
      // std::cout << joyInfo.axes[i];
      // std::cout << std::endl;
    }
    for (int i = 0; i < static_cast<int>(msg->buttons.size()); i++)
    {
      joyInfo.buttons[i] = msg->buttons[i];
      // std::cout << joyInfo.buttons[i];
      // std::cout << std::endl;
    }
  }

  void RLControllerBase::publishImuTransform(const sensor_msgs::msg::Imu &imu_msg)
  {
      geometry_msgs::msg::TransformStamped transformStamped;

      // Set header information
      transformStamped.header.stamp = get_node()->now();
      transformStamped.header.frame_id = "world_frame";    // The base frame of your robot
      transformStamped.child_frame_id = "base_link";     // The frame name for the IMU

      // Set translation (modify these values based on the physical IMU position on your robot)
      transformStamped.transform.translation.x = 0.0;
      transformStamped.transform.translation.y = 0.0;
      transformStamped.transform.translation.z = 0.0;

      // Set rotation from the IMU message
      transformStamped.transform.rotation = imu_msg.orientation;

      // Publish the transform
      tf_broadcaster_->sendTransform(transformStamped);
  }

    // Function to read CSV file and store data in a 2D vector
  std::vector<std::vector<double>> RLControllerBase::readCSV(const std::string& filePath) {
      std::vector<std::vector<double>> data;
      std::ifstream file(filePath);
      std::string line;

      while (std::getline(file, line)) {
          std::stringstream ss(line);
          std::string value;
          std::vector<double> row;
          while (std::getline(ss, value, ',')) {
              row.push_back(std::stod(value));
          }
          data.push_back(row);
      }
      return data;
  }
  // Function to calculate frame blending indices and factor
  void RLControllerBase::calcFrameBlend(double phase, int trajectory_num_frames, size_t& frame_idx0, size_t& frame_idx1, double& blend) {
      // Clip phase to be within 0.0 and 1.0
      phase = clamp(phase, 0.0, 1.0);
      // Calculate number of frames
      int num_frame = trajectory_num_frames - 1;
      // Calculate frame indices
      frame_idx0 = static_cast<size_t>(phase * num_frame);
      frame_idx1 = std::min(frame_idx0 + 1, static_cast<size_t>(num_frame));
      // Calculate blend factor
      blend = clamp((phase * num_frame) - static_cast<double>(frame_idx0), 0.0, 1.0);
  }

  // ========== 通用CSV加载函数实现 ==========
  std::vector<std::vector<double>> RLControllerBase::load_mocap_csv(
      const std::string& path,
      const std::vector<size_t>& indices,
      size_t stride)
  {
    std::vector<std::vector<double>> out;
    size_t row_idx = 0;
    for (const auto& row : this->readCSV(path)) {
      if ((stride > 1) && (row_idx % stride)) {
        ++row_idx;
        continue;
      }
      std::vector<double> frame;
      for (size_t i : indices)
        if (i < row.size()) frame.push_back(row[i]);
      if (!frame.empty()) out.push_back(std::move(frame));
      ++row_idx;
    }
    return out;
  }
  // ========== End ==========

  bool RLControllerBase::loadTestMotorParams()
  {
      try {
          // 获取配置文件路径
          std::string packageSharePath = ament_index_cpp::get_package_share_directory("rl_controllers");
          std::string yamlFilePath = packageSharePath + "/config/test_motor.yaml";

          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading test motor parameters from: %s", yamlFilePath.c_str());

          // 加载YAML文件
          YAML::Node config = YAML::LoadFile(yamlFilePath);

          // 获取test_motor节点 (处理ROS2参数格式 /**:/ros__parameters/test_motor)
          YAML::Node test_motor_node;
          if (config["/**"] && config["/**"]["ros__parameters"] && config["/**"]["ros__parameters"]["test_motor"]) {
              test_motor_node = config["/**"]["ros__parameters"]["test_motor"];
          } else if (config["test_motor"]) {
              test_motor_node = config["test_motor"];
          } else {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to find 'test_motor' node in YAML file");
              return false;
          }

          // 清空电机参数列表
          test_motor_params_.clear();

          // 读取每个电机的独立参数
          if (test_motor_node["motors"] && test_motor_node["motors"].IsSequence()) {
              for (const auto& motor_config : test_motor_node["motors"]) {
                  TestMotorParams params;
                  params.id = motor_config["id"].as<int>();
                  params.kp = motor_config["kp"].as<float>();
                  params.kd = motor_config["kd"].as<float>();
                  params.start_rad = motor_config["start_rad"].as<float>();
                  params.end_rad = motor_config["end_rad"].as<float>();
                  params.frequency = motor_config["frequency"].as<float>();
                  params.waveform_type = motor_config["waveform_type"].as<std::string>();
                  params.step_time = motor_config["step_time"].as<float>();
                  params.start_tau = motor_config["start_tau"].as<float>();
                  params.end_tau = motor_config["end_tau"].as<float>();
                  
                  // 读取 chirp 相关参数（可选，有默认值）
                  if (motor_config["f0_hz"]) {
                      params.f0_hz = motor_config["f0_hz"].as<float>();
                  }
                  if (motor_config["f1_hz"]) {
                      params.f1_hz = motor_config["f1_hz"].as<float>();
                  }
                  if (motor_config["T_sec"]) {
                      params.T_sec = motor_config["T_sec"].as<float>();
                  }
                  if (motor_config["phase"]) {
                      params.phase = motor_config["phase"].as<float>();
                  }
                  if (motor_config["ramp_sec"]) {
                      params.ramp_sec = motor_config["ramp_sec"].as<float>();
                  }

                  test_motor_params_.push_back(params);

                  if (params.waveform_type == "chirp") {
                      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Motor %d: kp=%.2f, kd=%.2f, rad=[%.3f,%.3f], wave=chirp, f0=%.2f Hz, f1=%.2f Hz, T=%.1f s",
                                 params.id, params.kp, params.kd, params.start_rad, params.end_rad,
                                 params.f0_hz, params.f1_hz, params.T_sec);
                  } else {
                      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Motor %d: kp=%.2f, kd=%.2f, rad=[%.3f,%.3f], freq=%.2f Hz, wave=%s",
                                 params.id, params.kp, params.kd, params.start_rad, params.end_rad,
                                 params.frequency, params.waveform_type.c_str());
                  }
              }
          } else {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No 'motors' array found in config file");
              return false;
          }

          if (test_motor_params_.empty()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No motors configured for testing");
              return false;
          }

          // 读取CSV保存路径（如果存在）
          if (test_motor_node["csv_save_path"]) {
              test_csv_save_path_ = test_motor_node["csv_save_path"].as<std::string>();
              // 如果路径不是绝对路径，则使用package路径
              if (test_csv_save_path_.empty() || test_csv_save_path_[0] != '/') {
                  test_csv_save_path_ = packageSharePath + "/" + test_csv_save_path_;
              }
          } else {
              // 默认保存路径
              test_csv_save_path_ = packageSharePath + "/test_data";
          }

          // 创建保存目录（如果不存在）
          if (!std::filesystem::exists(test_csv_save_path_)) {
              std::filesystem::create_directories(test_csv_save_path_);
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Created CSV save directory: %s", test_csv_save_path_.c_str());
          }

          // 读取decimation参数（如果存在）
          if (test_motor_node["decimation"]) {
              test_decimation_ = test_motor_node["decimation"].as<int>();
          } else {
              test_decimation_ = 10;  // 默认值
          }

          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test motor parameters loaded: %zu motors configured",
                     test_motor_params_.size());
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CSV save path: %s", test_csv_save_path_.c_str());
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Decimation: %d", test_decimation_);

          return true;

      } catch (const YAML::Exception& e) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load test motor parameters: %s", e.what());
          return false;
      } catch (const std::exception& e) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error loading test motor parameters: %s", e.what());
          return false;
      }
  }

  void RLControllerBase::handleTestMode()
  {
      // 高精度计时器
      auto current_time = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = current_time - test_start_time_;
      double time = elapsed.count(); // 真实经过的时间（秒）

      // 初始化测试状态（仅一次）
      if (!test_params_initialized_)
      {
          test_params_initialized_ = true;
          test_start_time_ = std::chrono::high_resolution_clock::now();
          test_count_ = 0;
          
          // 清空所有数据记录容器
          for (const auto& motor_params : test_motor_params_) {
              int motor_id = motor_params.id;
              test_time_values_[motor_id].clear();
              test_pos_des_values_[motor_id].clear();
              test_pos_current_values_[motor_id].clear();
              test_vel_current_values_[motor_id].clear();
              test_torque_planned_values_[motor_id].clear();
              test_torque_real_values_[motor_id].clear();
              test_kp_values_[motor_id].clear();
              test_kd_values_[motor_id].clear();
          }
          
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting test mode for %zu motors", test_motor_params_.size());
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Max test count: %d, Decimation: %d", MAX_TEST_COUNT, test_decimation_);
      }
      
      // 增加测试计数
      test_count_++;

      // 对所有关节进行控制
      for (size_t j = 0; j < hybridJointHandles_.size(); j++)
      {
          int current_motor_id = static_cast<int>(j) + 1;

          // 查找当前关节是否在测试电机列表中
          auto it = std::find_if(test_motor_params_.begin(), test_motor_params_.end(),
                                 [current_motor_id](const TestMotorParams& p) { return p.id == current_motor_id; });

          if (it != test_motor_params_.end())
          {
              // 这是一个测试电机，使用其独立参数
              const TestMotorParams& motor_params = *it;

              float tau_des = 0.0f;
              float pos_des = 0.0f;

              // 计算中心位置和摆动幅度
              float center = (motor_params.start_rad + motor_params.end_rad) / 2.0f;
              float amplitude = (motor_params.end_rad - motor_params.start_rad) / 2.0f;

              // 根据该电机的波形类型生成 pos_des
              if (motor_params.waveform_type == "sine")
              {
                  // 正弦波：在 start_rad 到 end_rad 之间摆动
                  pos_des = center + amplitude * std::sin(2 * M_PI * motor_params.frequency * time);
              }
              else if (motor_params.waveform_type == "fourier")
              {
                  // 傅里叶合成波
                  int N = 10;
                  pos_des = center;

                  std::default_random_engine generator;
                  std::uniform_real_distribution<float> amplitude_dist(0.1, 2.0);
                  std::uniform_real_distribution<float> phase_dist(0.0, 2 * M_PI);

                  for (int n = 1; n <= N; n++)
                  {
                      float random_amplitude_factor = amplitude_dist(generator);
                      float random_phase = phase_dist(generator);
                      pos_des += (2 * amplitude / (n * M_PI)) * random_amplitude_factor *
                                 std::sin(2 * M_PI * n * motor_params.frequency * time + random_phase);
                  }
              }
              else if (motor_params.waveform_type == "triangle")
              {
                  // 三角波：在 start_rad 到 end_rad 之间摆动
                  float period = 1.0f / motor_params.frequency;
                  float t = std::fmod(time, period);
                  float slope = 4 * amplitude / period;
                  if (t < period / 2)
                      pos_des = motor_params.start_rad + slope * t;
                  else
                      pos_des = motor_params.end_rad - slope * (t - period / 2);
              }
              else if (motor_params.waveform_type == "step")
              {
                  pos_des = 0.0f;  // step模式不使用位置
              }
              else if (motor_params.waveform_type == "linear")
              {
                  // 线性增长：从 start_rad 到 end_rad
                  pos_des = motor_params.start_rad + (motor_params.end_rad - motor_params.start_rad) * time * motor_params.frequency;
                  if (pos_des > motor_params.end_rad) pos_des = motor_params.end_rad;
              }
              else if (motor_params.waveform_type == "chirp")
              {
                  // 基本量
                  float center    = (motor_params.start_rad + motor_params.end_rad) * 0.5f;
                  float amplitude = (motor_params.end_rad   - motor_params.start_rad) * 0.5f;

                  // 当前扫频时间（不循环）：t_chirp ∈ [0, T]
                  double T  = std::max(0.001, (double)motor_params.T_sec);
                  double t_chirp = std::min(time, T);  // 扫频完成后停止在T

                  // 平台上限裁剪（可选：若你有全局 cap，可替换为你自己的）
                  double f0 = std::max(0.01, (double)motor_params.f0_hz);
                  double f1 = std::max(f0,   (double)motor_params.f1_hz);   // 保证 f1 ≥ f0
                  // 例如：f1 = std::min(f1, platform_cap_hz);

                  // 相位 & 包络
                  double phi = chirp_phase_lin(t_chirp, f0, f1, T) + motor_params.phase;
                  double env = envelope_lin(t_chirp, T, motor_params.ramp_sec);

                  // 生成位置指令：在 [start_rad, end_rad] 区间扫频
                  pos_des = center + amplitude * (float)(env * std::sin(phi));

                  // 防护（可选）：确保不越界
                  pos_des = std::clamp(pos_des,
                                       std::min(motor_params.start_rad, motor_params.end_rad),
                                       std::max(motor_params.start_rad, motor_params.end_rad));
              }
              else
              {
                  RCLCPP_WARN_THROTTLE(rclcpp::get_logger("rclcpp"), *get_node()->get_clock(), 1000,
                                       "Motor %d: 未知的波形类型：%s",
                                       motor_params.id, motor_params.waveform_type.c_str());
                  pos_des = 0.0f;
              }

              // step波形使用纯力矩控制，其他波形使用位置控制
              if (motor_params.waveform_type == "step")
              {
                  // 纯力矩控制：根据时间切换力矩
                  if (time < motor_params.step_time)
                      tau_des = motor_params.start_tau;
                  else
                      tau_des = motor_params.end_tau;

                  // 限制扭矩
                  tau_des = clamp(tau_des, -267.0f, 267.0f);

                  // 纯力矩控制：kp=0, kd=0
                  hybridJointHandles_[j]->setCommand(0.0f, 0.0f, 0.0f, 0.0f, tau_des);
              }
              else
              {
                  // 位置控制模式
                  hybridJointHandles_[j]->setCommand(pos_des, 0.0f, motor_params.kp, motor_params.kd, 0.0f);
              }
              
              // 数据记录：根据decimation参数采样
              if (test_count_ % test_decimation_ == 0)
              {
                  float pos_current = hybridJointHandles_[j]->getPosCurr();
                  float vel_current = hybridJointHandles_[j]->getVelCurr();
                  float real_tau = hybridJointHandles_[j]->getTauCurr();
                  
                  // 计算计划扭矩（用于位置控制模式）
                  if (motor_params.waveform_type != "step")
                  {
                      float position_error = pos_des - pos_current;
                      float velocity_error = 0.0f - vel_current;
                      tau_des = motor_params.kp * position_error + motor_params.kd * velocity_error;
                      tau_des = clamp(tau_des, -267.0f, 267.0f);
                  }
                  
                  // 记录数据
                  int motor_id = motor_params.id;
                  // chirp 波形：记录完整数据不受 MAX_TEST_COUNT 限制
                  // 其他波形：限制在 MAX_TEST_COUNT 内
                  bool should_record = (motor_params.waveform_type == "chirp") || 
                                      (static_cast<int>(test_time_values_[motor_id].size()) < MAX_TEST_COUNT);
                  
                  if (should_record)
                  {
                      test_time_values_[motor_id].push_back(time);
                      test_pos_des_values_[motor_id].push_back(pos_des);
                      test_pos_current_values_[motor_id].push_back(pos_current);
                      test_vel_current_values_[motor_id].push_back(vel_current);
                      test_torque_planned_values_[motor_id].push_back(tau_des);
                      test_torque_real_values_[motor_id].push_back(real_tau);
                      test_kp_values_[motor_id].push_back(motor_params.kp);
                      test_kd_values_[motor_id].push_back(motor_params.kd);
                  }
              }
          }
          else
          {
            // 其他关节保持当前位置，使用较小的阻尼防止晃动
            float hold_pos = hybridJointHandles_[j]->getPosCurr();
            hybridJointHandles_[j]->setCommand(hold_pos, 0.0f, 5.0f, 0.5f, 0.0f);
          }
      }
      
      // ========== Chirp 波形特殊处理：扫频完成后保存并退出 ==========
      // 检查是否有 chirp 波形
      bool has_chirp = false;
      bool chirp_completed = false;
      for (const auto& motor_params : test_motor_params_) {
          if (motor_params.waveform_type == "chirp") {
              has_chirp = true;
              double T = std::max(0.001, (double)motor_params.T_sec);
              if (time >= T) {
                  chirp_completed = true;
              }
              break;
          }
      }
      
      // 如果 chirp 扫频完成，立即保存 CSV 并退出 TEST 模式
      if (chirp_completed)
      {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Chirp sweep completed, saving data and exiting TEST mode...");
          
          // 为每个测试电机保存CSV文件
          for (const auto& motor_params : test_motor_params_)
          {
              int motor_id = motor_params.id;
              
              // 生成文件名
              std::ostringstream fileNameStream;
              fileNameStream << "motor_" << motor_id << "_response_"
                            << motor_params.waveform_type << "_";
              
              if (motor_params.waveform_type == "chirp") {
                  fileNameStream << "f" << std::fixed << std::setprecision(2) 
                                << motor_params.f0_hz << "to" << motor_params.f1_hz << "Hz_"
                                << motor_params.T_sec << "s_";
              } else {
                  fileNameStream << std::fixed << std::setprecision(2)
                                << motor_params.frequency << "Hz_";
              }
              
              fileNameStream << std::fixed << std::setprecision(2)
                            << motor_params.start_rad << "to"
                            << motor_params.end_rad << "rad_kp"
                            << motor_params.kp << "_kd"
                            << motor_params.kd << ".csv";
              
              std::string fileName = fileNameStream.str();
              std::string filePath = test_csv_save_path_ + "/" + fileName;
              
              // 保存数据
              std::ofstream csvFile(filePath);
              if (csvFile.is_open())
              {
                  // 写入CSV头部
                  csvFile << "Time,Target_Position,Current_Position,Current_Velocity,Torque_Command,Torque_Current,Kp,Kd\n";
                  
                  // 写入数据
                  size_t data_size = test_time_values_[motor_id].size();
                  for (size_t i = 0; i < data_size; i++)
                  {
                      csvFile << std::fixed << std::setprecision(6)
                          << test_time_values_[motor_id][i] << ","
                          << test_pos_des_values_[motor_id][i] << ","
                          << test_pos_current_values_[motor_id][i] << ","
                          << test_vel_current_values_[motor_id][i] << ","
                          << test_torque_planned_values_[motor_id][i] << ","
                          << test_torque_real_values_[motor_id][i] << ","
                          << test_kp_values_[motor_id][i] << ","
                          << test_kd_values_[motor_id][i] << "\n";
                  }
                  
                  csvFile.close();
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Chirp test data saved for motor %d: %s (%zu records)",
                             motor_id, filePath.c_str(), data_size);
                  
                  // 清空该电机的数据容器
                  test_time_values_[motor_id].clear();
                  test_pos_des_values_[motor_id].clear();
                  test_pos_current_values_[motor_id].clear();
                  test_vel_current_values_[motor_id].clear();
                  test_torque_planned_values_[motor_id].clear();
                  test_torque_real_values_[motor_id].clear();
                  test_kp_values_[motor_id].clear();
                  test_kd_values_[motor_id].clear();
              }
              else
              {
                  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open file %s for writing!", filePath.c_str());
              }
          }
          
          // 退出 TEST 模式，返回 DEFAULT 模式
          mode_ = Mode::DEFAULT;
          test_params_initialized_ = false;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Chirp test completed, returned to DEFAULT mode");
          return;  // 立即退出 handleTestMode
      }
      
      // ========== 其他波形：达到最大测试次数后保存并循环 ==========
      // 注意：chirp 波形不受 MAX_TEST_COUNT 限制，只按 T_sec 时长完成
      if (!has_chirp && test_count_ >= MAX_TEST_COUNT * test_decimation_)
      {
          // 为每个测试电机保存CSV文件
          for (const auto& motor_params : test_motor_params_)
          {
              int motor_id = motor_params.id;
              
              // 生成文件名
              std::ostringstream fileNameStream;
              fileNameStream << "motor_" << motor_id << "_response_"
                            << motor_params.waveform_type << "_"
                            << std::fixed << std::setprecision(2)
                            << motor_params.start_rad << "to"
                            << motor_params.end_rad << "rad_"
                            << motor_params.frequency << "Hz_kp"
                            << motor_params.kp << "_kd"
                            << motor_params.kd << ".csv";
              
              std::string fileName = fileNameStream.str();
              std::string filePath = test_csv_save_path_ + "/" + fileName;
              
              // 保存数据
              std::ofstream csvFile(filePath);
              if (csvFile.is_open())
              {
                  // 写入CSV头部
                  csvFile << "Time,Target_Position,Current_Position,Current_Velocity,Torque_Command,Torque_Current,Kp,Kd\n";
                  
                  // 写入数据
                  size_t data_size = test_time_values_[motor_id].size();
                  for (size_t i = 0; i < data_size; i++)
                  {
                      csvFile << std::fixed << std::setprecision(6)
                          << test_time_values_[motor_id][i] << ","
                          << test_pos_des_values_[motor_id][i] << ","
                          << test_pos_current_values_[motor_id][i] << ","
                          << test_vel_current_values_[motor_id][i] << ","
                          << test_torque_planned_values_[motor_id][i] << ","
                          << test_torque_real_values_[motor_id][i] << ","
                          << test_kp_values_[motor_id][i] << ","
                          << test_kd_values_[motor_id][i] << "\n";
                  }
                  
                  csvFile.close();
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test data saved for motor %d: %s (%zu records)",
                             motor_id, filePath.c_str(), data_size);
                  
                  // 清空该电机的数据容器
                  test_time_values_[motor_id].clear();
                  test_pos_des_values_[motor_id].clear();
                  test_pos_current_values_[motor_id].clear();
                  test_vel_current_values_[motor_id].clear();
                  test_torque_planned_values_[motor_id].clear();
                  test_torque_real_values_[motor_id].clear();
                  test_kp_values_[motor_id].clear();
                  test_kd_values_[motor_id].clear();
              }
              else
              {
                  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open file %s for writing!", filePath.c_str());
              }
          }
          
          // 重置测试计数和计时器
          test_count_ = 0;
          test_start_time_ = std::chrono::high_resolution_clock::now();
          
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test cycle completed, starting new cycle");
      }
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    legged::RLControllerBase, controller_interface::ControllerInterface)
