#include "AnkleSolver.h"

/*
 * 人形机器人踝关节解算器
 */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 将工具函数移到 AnkleSolverHelperFunctions.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern Eigen::Vector2d applyJointLimit(const Eigen::Vector2d& joint_pos,
                                        const Eigen::Vector2d& joint1_pos_range,
                                        const Eigen::Vector2d& joint2_pos_range);
extern Eigen::Vector2d applySixPointPolygonLimit(const Eigen::Vector2d& joint_pos);
extern double getMotorPosition(const std::string& motor_config,
                                const std::string& ankle_config,
                                const Eigen::Vector3d& r_Ai_0,
                                const Eigen::Vector3d& r_Bi_0,
                                const Eigen::Vector3d& r_Ci_0,
                                const double d_ankle,
                                const Eigen::Vector2d& ankle_pos,
                                const int motor_id);
extern Eigen::Matrix2d getJacobian(const std::string& motor_config,
                                    const std::string& ankle_config,
                                    const Eigen::Vector3d& r_A1_0,
                                    const Eigen::Vector3d& r_A2_0,
                                    const Eigen::Vector3d& r_B1_0,
                                    const Eigen::Vector3d& r_B2_0,
                                    const Eigen::Vector3d& r_C1_0,
                                    const Eigen::Vector3d& r_C2_0,
                                    const double d_ankle,
                                    const Eigen::Vector2d& ankle_pos,
                                    const Eigen::Vector2d& motor_pos);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 构造函数 (1 对应 脚踝左侧，2 对应 脚踝右侧)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HumanoidParallelAnkleSolver::HumanoidParallelAnkleSolver(
  const std::string& motor_config,
  const std::string& ankle_config,
  const Eigen::Vector3d& r_A1_0,
  const Eigen::Vector3d& r_A2_0,
  const Eigen::Vector3d& r_B1_0,
  const Eigen::Vector3d& r_B2_0,
  const Eigen::Vector3d& r_C1_0,
  const Eigen::Vector3d& r_C2_0,
  const double d_ankle,
  const Eigen::Vector2d& roll_range,
  const Eigen::Vector2d& pitch_range,
  const Eigen::Vector2d& left_motor_range,
  const Eigen::Vector2d& right_motor_range) {

if (motor_config != "X" && motor_config != "x" && motor_config != "Y" && motor_config != "y") {
  std::cout << "wrong motor configuration! please enter 'x' or 'X' or 'y' or 'Y' for motor_config" << std::endl;
  exit(1);
}

if (ankle_config != "RTP" && ankle_config != "rtp" && ankle_config != "PTR" && ankle_config != "ptr") {
  std::cout << "wrong ankle configuration! please enter 'rtp' or 'RTP' or 'ptr' or 'PTR' for ankle_config" << std::endl;
  exit(1);
}

std::cout << "\n---------- 初始化并联踝关节解算器 ----------"<< std::endl;

// 机械结构参数初始化
motor_config_ = motor_config;
ankle_config_ = ankle_config;

std::cout << "电机朝向：" << motor_config_ << std::endl;
std::cout << "踝关节属性：" << ankle_config_ << std::endl;

// 使用传入的参数初始化各关键点位置
r_A1_0_3d_ = r_A1_0;
r_A2_0_3d_ = r_A2_0;
r_B1_0_3d_ = r_B1_0;
r_B2_0_3d_ = r_B2_0;
r_C1_0_3d_ = r_C1_0;
r_C2_0_3d_ = r_C2_0;
d_ankle_ = d_ankle;

// 计算各连杆长度，运用空间向量的模计算长度
bar1_len_ = round((r_B1_0_3d_ - r_A1_0_3d_).norm() * 10000) / 10000;
bar2_len_ = round((r_B2_0_3d_ - r_A2_0_3d_).norm() * 10000) / 10000;
rod1_len_ = round((r_C1_0_3d_ - r_B1_0_3d_).norm() * 10000) / 10000;
rod2_len_ = round((r_C2_0_3d_ - r_B2_0_3d_).norm() * 10000) / 10000;

std::cout << "左电机：" << std::endl;
std::cout << "\tAB杆长度(m): " << bar1_len_ << std::endl;
std::cout << "\tBC杆长度(m): " << rod1_len_ << std::endl;
std::cout << "右电机：" << std::endl;
std::cout << "\tAB杆长度(m): " << bar2_len_ << std::endl;
std::cout << "\tBC杆长度(m): " << rod2_len_ << std::endl;

// 电机旋转轴
if (motor_config_ == "X" || motor_config_ == "x") {
    motor1_rotation_axis_ = {1, 0, 0};
    motor2_rotation_axis_ = {1, 0, 0};
} else if (motor_config_ == "Y" || motor_config_ == "y") {
    motor1_rotation_axis_ = {0, 1, 0};
    motor2_rotation_axis_ = {0, 1, 0};
}

// 正向运动学中牛顿-拉夫森迭代公式的初始猜测值
fk_max_iter_ = 5;            // 最大迭代次数
fk_error_tolerance_ = 1e-4;   // 迭代误差容限
fk_init_ankle_pos_ = {0, 0};  // 初始猜测值
fk_prev_ankle_pos_ = {0, 0};  // 上一次的脚踝位置

// Roll和Pitch的限制范围
roll_range_ = roll_range;
pitch_range_ = pitch_range;

// 电机角度限制
motor1_range_ = left_motor_range;
motor2_range_ = right_motor_range;

std::cout << "电机角度限制[min, max]：" << std::endl;
std::cout << "\t左侧电机(motor1):" << std::endl;
std::cout << "\t\t(rad)：[" << motor1_range_(0) << ", " << motor1_range_(1) << "]" << std::endl;
std::cout << "\t\t(deg)：[" << motor1_range_(0) * RAD_TO_DEG << ", " << motor1_range_(1) * RAD_TO_DEG << "]" << std::endl;
std::cout << "\t右侧电机(motor2):" << std::endl;
std::cout << "\t\t(rad)：[" << motor2_range_(0) << ", " << motor2_range_(1) << "]" << std::endl;
std::cout << "\t\t(deg)：[" << motor2_range_(0) * RAD_TO_DEG << ", " << motor2_range_(1) * RAD_TO_DEG << "]" << std::endl;

std::cout << "踝关节角度限制[min, max]：" << std::endl;
std::cout << "\tRoll:" << std::endl;
std::cout << "\t\t(rad)：[" << roll_range_(0) << ", " << roll_range_(1) << "]" << std::endl;
std::cout << "\t\t(deg)：[" << roll_range_(0) * RAD_TO_DEG << ", " << roll_range_(1) * RAD_TO_DEG << "]" << std::endl;
std::cout << "\tPitch:" << std::endl;
std::cout << "\t\t(rad)：[" << pitch_range_(0) << ", " << pitch_range_(1) << "]" << std::endl;
std::cout << "\t\t(deg)：[" << pitch_range_(0) * RAD_TO_DEG << ", " << pitch_range_(1) * RAD_TO_DEG << "]" << std::endl;

// 初始化数据偏置
realToSolverMotorPosOffsets_ = Eigen::Vector2d::Zero();
solverToRealJointPosOffsets_ = Eigen::Vector2d::Zero();
// 初始化脚踝解算数据
data_ = AnkleSolveDebugDataStru();
clearAnkleSolverDebugData();

std::cout << "\n---------- 初始化并联踝关节解算器完成 ----------"<< std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 类函数
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 清空脚踝解算数据
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HumanoidParallelAnkleSolver::clearAnkleSolverDebugData() {
  // 当前控制循环的电机输入数据是否准备好
  data_.isRealInputMotorDataReady_ = false;
  // READ函数: motor to joint 结果（脚踝电机（左连杆电机，右连杆电机）信息转换为脚踝roll pitch信息）
  data_.motorToJoint_ = AnkleMotorToJointDataStru();
  data_.motorToJoint_.realMotorInput_ = AnkleDataStru();
  data_.motorToJoint_.solverMotorInput_ = AnkleDataStru();
  data_.motorToJoint_.solverStatus_ = AnkleMotorToJointStatusStru();
  data_.motorToJoint_.solverJointOutput_ = AnkleDataStru();
  data_.motorToJoint_.realJointOutput_ = AnkleDataStru();
  // WRITE函数: joint to motor 结果 （脚踝 roll pitch信息转换为脚踝电机（左连杆电机，右连杆电机）信息）
  data_.jointToMotor_ = AnkleJointToMotorDataStru();
  data_.jointToMotor_.realJointInput_ = AnkleDataStru();
  data_.jointToMotor_.solverJointInput_ = AnkleDataStru();
  data_.jointToMotor_.solverMotorOutput_ = AnkleDataStru();
  data_.jointToMotor_.realMotorOutput_ = AnkleDataStru();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 逆向运动学运算
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d HumanoidParallelAnkleSolver::inverseKinematics(const Eigen::Vector2d& ankle_pos) {
  // 创建局部变量存储关节角度
  Eigen::Vector2d limited_ankle_pos;
  if (APPLY_POLYGON_LIMIT) {
    limited_ankle_pos = applySixPointPolygonLimit(ankle_pos);
  } else {
    limited_ankle_pos = applyJointLimit(ankle_pos, roll_range_, pitch_range_);
  }

  // 左侧踝关节电机角度
  double motor1_pos = getMotorPosition(
    motor_config_,
    ankle_config_,
    r_A1_0_3d_,
    r_B1_0_3d_,
    r_C1_0_3d_,
    d_ankle_,
    limited_ankle_pos,
    1
  );
  // 右侧踝关节电机角度
  double motor2_pos = getMotorPosition(
    motor_config_,
    ankle_config_,
    r_A2_0_3d_,
    r_B2_0_3d_,
    r_C2_0_3d_,
    d_ankle_,
    limited_ankle_pos,
    2
  );

  // 计算电机角度
  Eigen::Vector2d motor_pos = {motor1_pos, motor2_pos};
  return motor_pos;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 逆向运动学运算（雅可比矩阵）
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::pair<Eigen::Vector2d, Eigen::MatrixXd> HumanoidParallelAnkleSolver::inverseKinematicsWithJacobian(const Eigen::Vector2d& ankle_pos) {
  // 应用关节限制
  Eigen::Vector2d limited_ankle_pos;
  if (APPLY_POLYGON_LIMIT) {
    limited_ankle_pos = applySixPointPolygonLimit(ankle_pos);
  } else {
    limited_ankle_pos = applyJointLimit(ankle_pos, roll_range_, pitch_range_);
  }

  // 计算电机角度
  Eigen::Vector2d motor_pos = inverseKinematics(ankle_pos);

  // 计算雅可比矩阵 - 使用限制后的踝关节角度
  Eigen::MatrixXd Jacobian = getJacobian(
                                motor_config_,
                                ankle_config_,
                                r_A1_0_3d_, r_A2_0_3d_,
                                r_B1_0_3d_, r_B2_0_3d_,
                                r_C1_0_3d_, r_C2_0_3d_,
                                d_ankle_,
                                limited_ankle_pos,
                                motor_pos
                              );

  return {motor_pos, Jacobian};
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 正向运动学运算
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<Eigen::Vector2d, int, bool, double> HumanoidParallelAnkleSolver::forwardKinematics(const Eigen::Vector2d& motor_pos) {
  // 创建局部变量存储电机角度
  Eigen::Vector2d limited_motor_pos = applyJointLimit(motor_pos, motor1_range_, motor2_range_);

  Eigen::Vector2d ankle_pos_k = fk_init_ankle_pos_;  // 更新初始猜测值
  int num_iterations = 0;                            // 记录迭代次数
  double final_error = 0;                            // 记录最终误差

  while (num_iterations < fk_max_iter_) {
    auto [calculated_motor_pos, Jacobian] = inverseKinematicsWithJacobian(ankle_pos_k);
    // 计算误差：运动学论文中定义的公式 21
    Eigen::Vector2d error = calculated_motor_pos - limited_motor_pos;
    final_error = error.norm();

    // 提示误差已收敛
    if (error.norm() < fk_error_tolerance_) {
      break;  // Converged: Error below tolerance
    }
    // 提示雅可比矩阵接近奇异点
    Eigen::MatrixXd Jacobian_inv = Jacobian.completeOrthogonalDecomposition().pseudoInverse();
    if (Jacobian_inv.norm() < 1e-12) {
      std::cerr << "Jacobian matrix is near-singular." << std::endl;
    }

    // 牛顿-拉夫森迭代公式：运动学论文中定义的公式 23
    ankle_pos_k = ankle_pos_k - Jacobian_inv * error;
    num_iterations++;
  }

  // 提示已达到最大迭代值
  if (num_iterations == fk_max_iter_) {
    std::cout << "Stopped: Maximum iterations reached." << std::endl;
    fk_prev_ankle_pos_ = ankle_pos_k;
    if (APPLY_POLYGON_LIMIT) {
      fk_prev_ankle_pos_ = applySixPointPolygonLimit(fk_prev_ankle_pos_);
    } else {
      fk_prev_ankle_pos_ = applyJointLimit(fk_prev_ankle_pos_, roll_range_, pitch_range_);
    }
    return {fk_prev_ankle_pos_, num_iterations, false, final_error};
  }

  // 更新上一次的脚踝位置
  fk_prev_ankle_pos_ = ankle_pos_k;
  return {ankle_pos_k, num_iterations, true, final_error};
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 电机数据 -> 脚踝关节数据
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, int, bool, double> HumanoidParallelAnkleSolver::motorToJoint(
  const Eigen::Vector2d& motor_pos,
  const Eigen::Vector2d& motor_vel,
  const Eigen::Vector2d& motor_torque) {
// 计算脚踝关节角度
auto [ankle_pos, num_iterations, isSuccess, final_error] = forwardKinematics(motor_pos);

// 计算当前给定电机角度下的雅可比矩阵
auto [calculated_motor_pos, Jacobian] = inverseKinematicsWithJacobian(ankle_pos);

// 计算雅可比矩阵的伪逆
Eigen::MatrixXd Jacobian_inv = Jacobian.completeOrthogonalDecomposition().pseudoInverse();

// 计算脚踝关节速度
Eigen::Vector2d ankle_vel = Jacobian_inv * motor_vel;

// 计算脚踝关节扭矩
Eigen::Vector2d ankle_torque = Jacobian.transpose() * motor_torque;
return {ankle_pos, ankle_vel, ankle_torque, num_iterations, isSuccess, final_error};
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 脚踝关节数据 -> 电机数据
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d> HumanoidParallelAnkleSolver::jointToMotor(
  const Eigen::Vector2d& ankle_pos,
  const Eigen::Vector2d& ankle_vel,
  const Eigen::Vector2d& ankle_torque) {
// 计算电机角度和雅可比矩阵
auto [motor_pos, Jacobian] = inverseKinematicsWithJacobian(ankle_pos);

// 计算雅可比矩阵的伪逆
Eigen::MatrixXd Jacobian_inv = Jacobian.completeOrthogonalDecomposition().pseudoInverse();

// 计算电机速度：motor_vel = Jacobian * ankle_vel
Eigen::Vector2d motor_vel = Jacobian * ankle_vel;

// 计算电机扭矩：motor_torque = (Jacobian^T)^{-1} * ankle_torque
Eigen::Vector2d motor_torque = Jacobian_inv.transpose() * ankle_torque;

return {motor_pos, motor_vel, motor_torque};
}