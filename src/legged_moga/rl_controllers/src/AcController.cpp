#include "rl_controllers/AcController.h"
#include "rl_controllers/RotationTools.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace legged {

void AcController::handleCmdVelTimeout() {
  // 计算本周期时间间隔 dt
  static rclcpp::Time last_handle_time = get_node()->now();
  rclcpp::Time now = get_node()->now();
  double dt = (now - last_handle_time).seconds();
  last_handle_time = now;

  // 检查 /cmd_vel 超时
  double time_since_last_cmd = (now - last_cmd_vel_time_).seconds();
  if (time_since_last_cmd > cmd_vel_timeout_duration_) {
    // 如果当前内部速度命令不为零，则执行减速
    if (internal_command_.linear.x != 0.0 ||
        internal_command_.linear.y != 0.0 ||
        internal_command_.angular.z != 0.0)
    {
      // 计算本周期减速量
      double decel = cmd_vel_deceleration_rate_ * dt;

      // X 方向线性减速
      if (internal_command_.linear.x > 0.0) {
        internal_command_.linear.x = std::max(0.0, internal_command_.linear.x - decel);
      } else if (internal_command_.linear.x < 0.0) {
        internal_command_.linear.x = std::min(0.0, internal_command_.linear.x + decel);
      }

      // Y 方向线性减速
      if (internal_command_.linear.y > 0.0) {
        internal_command_.linear.y = std::max(0.0, internal_command_.linear.y - decel);
      } else if (internal_command_.linear.y < 0.0) {
        internal_command_.linear.y = std::min(0.0, internal_command_.linear.y + decel);
      }

      // Yaw 方向线性减速
      if (internal_command_.angular.z > 0.0) {
        internal_command_.angular.z = std::max(0.0, internal_command_.angular.z - decel);
      } else if (internal_command_.angular.z < 0.0) {
        internal_command_.angular.z = std::min(0.0, internal_command_.angular.z + decel);
      }
    }
  }
}

void AcController::handleWalkMode() {
  // 处理 cmd_vel 超时减速
  handleCmdVelTimeout();

  // 将更新后的 internal_command_ 作为最终速度命令
  // command_.x = internal_command_.linear.x;
  // command_.y = internal_command_.linear.y;
  // command_.yaw = internal_command_.angular.z;

  // compute observation & actions
  // if (loopCount_ % robotCfg_.controlCfg.decimation == 0) {
  //   computeObservation();
  //   computeActions();

  //   // limit action range
  //   scalar_t actionMin = -robotCfg_.clipActions;
  //   scalar_t actionMax = robotCfg_.clipActions;
  //   std::transform(actions_.begin(), actions_.end(), actions_.begin(),
  //                  [actionMin, actionMax](scalar_t x) { return std::max(actionMin, std::min(actionMax, x)); });
  // }

  // for (int i = 0; i < actionsSize_; i++)
  // {
  //   std::string partName = joint_names_[leg_joint_mapping[i]];
  //   scalar_t pos_des = actions_[i] * robotCfg_.controlCfg.actionScale + defaultJointAngles_(leg_joint_mapping[i]);
  //   double stiffness = robotCfg_.controlCfg.stiffness[partName]; // 根据关节名称获取刚度
  //   double damping = robotCfg_.controlCfg.damping[partName]; // 根据关节名称获取阻尼
  //   hybridJointHandles_[leg_joint_mapping[i]]->setCommand(pos_des, 0, stiffness, damping, 0);
  //   lastActions_(i, 0) = actions_[i];
  // }

  //   for (int i = 0; i < joint_mapping_fixed.size(); i++)
  // {
  //     std::string partName = joint_names_[joint_mapping_fixed[i]];
  //     double stiffness = robotCfg_.controlCfg.stiffness[partName]; // 根据关节名称获取刚度
  //     double damping = robotCfg_.controlCfg.damping[partName];     // 根据关节名称获取阻尼
  //     static double delta_pos_threshord = 0.2;
  //     double real_ref_pos = defaultJointAngles_(joint_mapping_fixed[i]);
  //     double delta_pos = real_ref_pos - allJointPos_(joint_mapping_fixed[i]);
  //     double send_ref_pos = real_ref_pos;
  //     if(delta_pos > 0){
  //       send_ref_pos = std::min(real_ref_pos, allJointPos_(joint_mapping_fixed[i]) + delta_pos_threshord);
  //     }
  //     else
  //     {
  //       send_ref_pos = std::max(real_ref_pos, allJointPos_(joint_mapping_fixed[i]) - delta_pos_threshord);
  //     } 
  //     hybridJointHandles_[joint_mapping_fixed[i]]->setCommand(send_ref_pos, 0, stiffness, damping, 0);
  // }

  //   for (int i = 0; i < swing_arm_mapping.size(); i++)
  // {
  //   std::string partName = joint_names_[swing_arm_mapping[i]];
  //   double stiffness = robotCfg_.controlCfg.stiffness[partName]; // 根据关节名称获取刚度
  //   double damping = robotCfg_.controlCfg.damping[partName];     // 根据关节名称获取阻尼
  //   static float alpha = 0.4;
  //   static double delta_pos_threshord = 0.1;

  //   // double real_ref_pos = 0.0;

  //   double real_ref_pos = ref_dof_pos_cal[i] * (alpha + (1-alpha) * std::abs(command_.x /2.4));
  //   double delta_pos = real_ref_pos - allJointPos_(swing_arm_mapping[i]);
  //   double send_ref_pos = real_ref_pos;
  //   if(delta_pos > 0){
  //     send_ref_pos = std::min(real_ref_pos, allJointPos_(swing_arm_mapping[i]) + delta_pos_threshord);
  //   }
  //   else
  //   {
  //     send_ref_pos = std::max(real_ref_pos, allJointPos_(swing_arm_mapping[i]) - delta_pos_threshord);
  //   } 
  //   hybridJointHandles_[swing_arm_mapping[i]]->setCommand(send_ref_pos, 0, stiffness, damping, 0);
  // }
}

bool AcController::loadModel() {
  std::string policyFilePath;
  if (!get_node()->get_parameter("policyFile", policyFilePath)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rlccpp"), "Get policy path fail from param server, some error occur!");
    return false;
  }

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("rl_controllers");
  std::string fullPolicyFilePath = package_share_directory + "/" + policyFilePath;

  policyFilePath_ = fullPolicyFilePath;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rlccpp"), "Load Onnx model from path : " << policyFilePath);

  // create env
  onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
  // create session
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetInterOpNumThreads(1);
  sessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, fullPolicyFilePath.c_str(), sessionOptions);
  // get input and output info
  inputNames_.clear();
  outputNames_.clear();
  inputShapes_.clear();
  outputShapes_.clear();
  Ort::AllocatorWithDefaultOptions allocator;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rlccpp"),"count: " << sessionPtr_->GetOutputCount());
  for (int i = 0; i < static_cast<int>(sessionPtr_->GetInputCount()); i++) {
    auto inputnamePtr = sessionPtr_->GetInputNameAllocated(i, allocator);
    inputNodeNameAllocatedStrings.push_back(std::move(inputnamePtr));
    inputNames_.push_back(inputNodeNameAllocatedStrings.back().get());
    // inputNames_.push_back(sessionPtr_->GetInputNameAllocated(i, allocator).get());
    inputShapes_.push_back(sessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    std::cout << "Input Name [" << i << "]: " << inputnamePtr.get() << std::endl;
  }
  for (int i = 0; i < static_cast<int>(sessionPtr_->GetOutputCount()); i++) {
    auto outputnamePtr = sessionPtr_->GetOutputNameAllocated(i, allocator);
    outputNodeNameAllocatedStrings.push_back(std::move(outputnamePtr));
    outputNames_.push_back(outputNodeNameAllocatedStrings.back().get());
    // outputNames_.push_back(sessionPtr_->GetOutputNameAllocated(i, allocator).get());
    std::cout << sessionPtr_->GetOutputNameAllocated(i, allocator).get() << std::endl;
    outputShapes_.push_back(sessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    std::cout << "Output Name [" << i << "]: " << outputnamePtr.get() << std::endl;
  }

  // 打印 inputNames_
  std::cout << "Input Names:" << std::endl;
  for (const char* name : inputNames_) {
      std::cout << name << std::endl;
  }

  // 打印 outputNames_
  std::cout << "Output Names:" << std::endl;
  for (const char* name : outputNames_) {
      std::cout << name << std::endl;
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rlccpp"), "Load Onnx model successfully !!!");
  return true;
}

bool AcController::loadRLCfg() {
  RLRobotCfg::InitState& initState = robotCfg_.initState;
  RLRobotCfg::armInitState &arminitState = robotCfg_.arminitState;
  RLRobotCfg::waistInitState &waistinitState = robotCfg_.waistinitState;
  RLRobotCfg::ControlCfg& controlCfg = robotCfg_.controlCfg;
  RLRobotCfg::ObsScales& obsScales = robotCfg_.obsScales;

  int error = 0;
  // MOGA Left Leg
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.left_hip_pitch", initState.left_hip_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.left_hip_roll", initState.left_hip_roll));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.left_hip_yaw", initState.left_hip_yaw));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.left_knee_pitch", initState.left_knee_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.left_ankle_pitch", initState.left_ankle_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.left_ankle_roll", initState.left_ankle_roll));

  // MOGA Right Leg
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.right_hip_pitch", initState.right_hip_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.right_hip_roll", initState.right_hip_roll));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.right_hip_yaw", initState.right_hip_yaw));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.right_knee_pitch", initState.right_knee_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.right_ankle_pitch", initState.right_ankle_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_joint_angle.right_ankle_roll", initState.right_ankle_roll));

  // MOGA Waist
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_waist_joint_angle.0waist_yaw", waistinitState.waist_yaw));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_waist_joint_angle.waist_pitch", waistinitState.waist_pitch));

  // MOGA Left Arm
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.1left_shoulder_pitch", arminitState.left_shoulder_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.1left_shoulder_roll", arminitState.left_shoulder_roll));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.left_shoulder_yaw", arminitState.left_shoulder_yaw));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.1left_elbow_pitch", arminitState.left_elbow_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.left_elbow_yaw", arminitState.left_elbow_yaw));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.1left_wrist_pitch", arminitState.left_wrist_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.left_wrist_roll", arminitState.left_wrist_roll));

  // MOGA Right Arm
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.1right_shoulder_pitch", arminitState.right_shoulder_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.1right_shoulder_roll", arminitState.right_shoulder_roll));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.right_shoulder_yaw", arminitState.right_shoulder_yaw));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.1right_elbow_pitch", arminitState.right_elbow_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.right_elbow_yaw", arminitState.right_elbow_yaw));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.1right_wrist_pitch", arminitState.right_wrist_pitch));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_arm_joint_angle.right_wrist_roll", arminitState.right_wrist_roll));

  // MOGA Head (add if needed in the struct)
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_head_joint_angle.head_yaw", robotCfg_.headInitState.head_yaw));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.init_state.default_head_joint_angle.head_pitch", robotCfg_.headInitState.head_pitch));


  error += static_cast<int>(!get_node()->get_parameters("LeggedRobotCfg.control.stiffness", controlCfg.stiffness));
  error += static_cast<int>(!get_node()->get_parameters("LeggedRobotCfg.control.damping", controlCfg.damping));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.control.action_scale", controlCfg.actionScale));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.control.decimation", controlCfg.decimation));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.control.cycle_time", controlCfg.cycle_time));

  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.normalization.clip_scales.clip_observations", robotCfg_.clipObs));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.normalization.clip_scales.clip_actions", robotCfg_.clipActions));

  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.normalization.obs_scales.lin_vel", obsScales.linVel));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.normalization.obs_scales.ang_vel", obsScales.angVel));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.normalization.obs_scales.dof_pos", obsScales.dofPos));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.normalization.obs_scales.dof_vel", obsScales.dofVel));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.normalization.obs_scales.height_measurements", obsScales.heightMeasurements));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.normalization.obs_scales.quat", obsScales.quat));

  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.size.actions_size", actionsSize_));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.size.observations_size", observationSize_));

  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.size.num_hist", num_hist_));

  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.mode.sw_mode", sw_mode_));
  error += static_cast<int>(!get_node()->get_parameter("LeggedRobotCfg.mode.cmd_threshold", cmd_threshold_));

  error += static_cast<int>(!get_node()->get_parameter("use_sim_handles", use_sim_handles_));

  error += static_cast<int>(!get_node()->get_parameter("init_pos_control.lie_kp_large", lie_kp_large_));
  error += static_cast<int>(!get_node()->get_parameter("init_pos_control.lie_kd_large", lie_kd_large_));
  error += static_cast<int>(!get_node()->get_parameter("init_pos_control.lie_kp_ankle", lie_kp_ankle_));
  error += static_cast<int>(!get_node()->get_parameter("init_pos_control.lie_kd_ankle", lie_kd_ankle_));
  error += static_cast<int>(!get_node()->get_parameter("init_pos_control.stance_kp_large", stance_kp_large_));
  error += static_cast<int>(!get_node()->get_parameter("init_pos_control.stance_kd_large", stance_kd_large_));
  error += static_cast<int>(!get_node()->get_parameter("init_pos_control.stance_kp_ankle", stance_kp_ankle_));
  error += static_cast<int>(!get_node()->get_parameter("init_pos_control.stance_kd_ankle", stance_kd_ankle_));

  // 加载 cmd_vel 超时减速相关参数
  error += static_cast<int>(!get_node()->get_parameter("cmd_vel_timeout_duration", cmd_vel_timeout_duration_));
  error += static_cast<int>(!get_node()->get_parameter("cmd_vel_deceleration_rate", cmd_vel_deceleration_rate_));

  // 简化矢量初始化
  actions_ = std::vector<tensor_element_t>(actionsSize_, 0.0f);
  observations_ = std::vector<tensor_element_t>(observationSize_ * num_hist_, 0.0f);

  command_.x = 0;
  command_.y = 0;
  command_.yaw = 0;
  baseLinVel_.setZero();
  basePosition_.setZero();
     std::vector<scalar_t> defaultJointAngles{
         // MOGA Left Leg
         robotCfg_.initState.left_hip_pitch, robotCfg_.initState.left_hip_roll, robotCfg_.initState.left_hip_yaw,
         robotCfg_.initState.left_knee_pitch, robotCfg_.initState.left_ankle_pitch, robotCfg_.initState.left_ankle_roll,
         // MOGA Right Leg
         robotCfg_.initState.right_hip_pitch, robotCfg_.initState.right_hip_roll, robotCfg_.initState.right_hip_yaw,
         robotCfg_.initState.right_knee_pitch, robotCfg_.initState.right_ankle_pitch, robotCfg_.initState.right_ankle_roll,
         // MOGA Waist
         robotCfg_.waistinitState.waist_yaw, robotCfg_.waistinitState.waist_pitch,
         // MOGA Left Arm
         robotCfg_.arminitState.left_shoulder_pitch, robotCfg_.arminitState.left_shoulder_roll, robotCfg_.arminitState.left_shoulder_yaw,
         robotCfg_.arminitState.left_elbow_pitch, robotCfg_.arminitState.left_elbow_yaw,
         robotCfg_.arminitState.left_wrist_pitch, robotCfg_.arminitState.left_wrist_roll,
        // MOGA Right Arm
        robotCfg_.arminitState.right_shoulder_pitch, robotCfg_.arminitState.right_shoulder_roll, robotCfg_.arminitState.right_shoulder_yaw,
        robotCfg_.arminitState.right_elbow_pitch, robotCfg_.arminitState.right_elbow_yaw,
        robotCfg_.arminitState.right_wrist_pitch, robotCfg_.arminitState.right_wrist_roll,
        // MOGA Head
        robotCfg_.headInitState.head_yaw, robotCfg_.headInitState.head_pitch
        };

  // 简化Eigen矢量初始化
  lastActions_ = Eigen::VectorXd::Zero(actionsSize_);
  const int inputSize = num_hist_ * observationSize_;
  proprioHistoryBuffer_ = Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1>::Zero(inputSize);
  defaultJointAngles_ = Eigen::VectorXd::Zero(actuatedDofNum_);
  defaultJointAnglesActuated_ = Eigen::VectorXd::Zero(actionsSize_);

  for (int i = 0; i < actuatedDofNum_; i++) {
    defaultJointAngles_(i) = defaultJointAngles[i];
  }
  for (int i = 0; i < actionsSize_; i++){
    defaultJointAnglesActuated_(i) = defaultJointAngles[leg_joint_mapping[i]];
  }

  return (error == 0);
}

void AcController::computeActions() {
  // create input tensor object
  std::vector<Ort::Value> inputValues;

  try {
    inputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, observations_.data(), observations_.size(),
                                                                     inputShapes_[0].data(), inputShapes_[0].size()));
  } catch (const std::exception& e) {
    return;
  }

  // run inference
  Ort::RunOptions runOptions;

  try {
    std::vector<Ort::Value> outputValues = sessionPtr_->Run(runOptions, inputNames_.data(), inputValues.data(), 1, outputNames_.data(), 1);

    for (int i = 0; i < actionsSize_; i++) {
      actions_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
    }
  } catch (const std::exception& e) {
    return;
  }
}

void AcController::computeObservation() {
  // command
  vector_t command(5);

  if (sw_mode_)
  {
    cmd_norm_ = std::sqrt(command_.x * command_.x + command_.y * command_.y + command_.yaw * command_.yaw);
    if (cmd_norm_ <= cmd_threshold_ && phase_ <=0.05 || phase_ >= 0.95)
    {
      auto now = rclcpp::Clock(RCL_ROS_TIME).now();
      double now_sec = now.seconds();
      current_time_ = now_sec;
      phase_ = 0;
      phase_start_time_ = now_sec; // Record the current time as the start time
    }
    else
    {
      auto now = rclcpp::Clock(RCL_ROS_TIME).now();
      double now_sec = now.seconds();

      // Calculate phase increment based on the elapsed time since phase was reset
      phase_ = now_sec - phase_start_time_;
    }
  }

  double arm_phase = std::fmod((phase_ + 0.2) / robotCfg_.controlCfg.cycle_time, 1.0);

  // Normalize phase to [0, 1)
  phase_ = std::fmod(phase_ / robotCfg_.controlCfg.cycle_time, 1.0);

  // 声明局部变量，使用size_t类型避免符号比较警告
  size_t frame_idx0, frame_idx1;
  double blend;
  calcFrameBlend(arm_phase, trajectory_num_frames, frame_idx0, frame_idx1, blend);

  if (ref_dof_pos.empty()) {
    return;
  }

  if (frame_idx0 >= ref_dof_pos.size() || frame_idx1 >= ref_dof_pos.size()) {
    return;
  }

  ref_dof_pos_low = ref_dof_pos[frame_idx0];
  ref_dof_pos_high = ref_dof_pos[frame_idx1];

  for (int i = 0; i < int(ref_dof_pos[0].size()); i++) {
      ref_dof_pos_cal[i] = (1 - blend) * ref_dof_pos_low[i] + blend * ref_dof_pos_high[i];
  }

  // last command
  last_walkmode_ = walkmode_;

  std_msgs::msg::Float64 phase_msg;
  phase_msg.data = phase_;
  phasePublisher_->publish(phase_msg);

  command[0] = sin(2 * M_PI * phase_);
  command[1] = cos(2 * M_PI * phase_);
  command[2] = command_.x;
  command[3] = command_.y;
  command[4] = command_.yaw;

  // actions
  vector_t actions(lastActions_);

  RLRobotCfg::ObsScales& obsScales = robotCfg_.obsScales;
  matrix_t commandScaler = Eigen::DiagonalMatrix<scalar_t, 3>(obsScales.linVel, obsScales.linVel, obsScales.angVel);

  vector_t proprioObs(observationSize_);

  try {
    proprioObs << command, // 5
        (propri_.jointPos.segment(12, 1) - defaultJointAngles_.segment(12, 1)) * obsScales.dofPos,  // 12
        (propri_.jointPos.segment(0, 12) - defaultJointAngles_.segment(0, 12)) * obsScales.dofPos,  // 12
        propri_.jointVel.segment(0, 12) * obsScales.dofVel,  // 12
        actions,  // 12
        propri_.baseAngVel * obsScales.angVel,  // 3
        propri_.baseEulerXyz * obsScales.quat;  // 3
  } catch (const std::exception& e) {
    return;
  }

  if (isfirstRecObs_)
    {
      for (
          int i = 30; i < 42; i++)
      {
        /* code */
        proprioObs(i, 0) = 0.0;
      }

      for (int i = 0; i < num_hist_; i++)
      {
        proprioHistoryBuffer_.segment(i * observationSize_, observationSize_) = proprioObs.cast<tensor_element_t>();
      }
      isfirstRecObs_ = false;
    }

  proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) =
      proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - observationSize_);
  proprioHistoryBuffer_.tail(observationSize_) = proprioObs.cast<tensor_element_t>();
  // clang-format on

  for (int i = 0; i < (observationSize_ * num_hist_); i++){
    observations_[i] = static_cast<tensor_element_t>(proprioHistoryBuffer_[i]);
  }

  // Limit observation range
  scalar_t obsMin = -robotCfg_.clipObs;
  scalar_t obsMax = robotCfg_.clipObs;
  std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                 [obsMin, obsMax](scalar_t x) { return std::max(obsMin, std::min(obsMax, x)); });
}

}  // namespace legged


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  legged::AcController, controller_interface::ControllerInterface)

