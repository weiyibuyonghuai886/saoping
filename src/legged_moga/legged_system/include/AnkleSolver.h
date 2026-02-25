#ifndef HUMANOID_PARALLEL_ANKLE_SOLVER_H_
#define HUMANOID_PARALLEL_ANKLE_SOLVER_H_

/**
 * @file AnkleSolver.h
 * @brief 并联踝关节解算器头文件
 */

#include <Eigen/Dense>
#include <tuple>
#include <utility>
#include <vector>
#include <cmath>
#include <iostream>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 常量
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define APPLY_POLYGON_LIMIT 1  // 是否应用六边形工作区域限制
const double RAD_TO_DEG = 180.0 / M_PI;
const double DEG_TO_RAD = M_PI / 180.0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 实机控制数据结构体
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 踝关节数据结构体
struct AnkleDataStru
{
  Eigen::Vector2d pos_, vel_, tau_;  // 关节角度、速度、扭矩 (roll_joint, pitch_joint) 或者 (left_motor, right_motor)
};

struct AnkleMotorToJointStatusStru
{
  bool isSuccess_;              // 迭代是否成功（最终解是否在指定误差范围内）
  int iterationNum_;            // 迭代次数
  double finalError_;           // 最终电机位置迭代误差(rad)
};

struct AnkleMotorToJointDataStru
{
  AnkleDataStru realMotorInput_;                            // 原始电机输入数据(左,右电机位置)(左,右电机速度)(左,右电机扭矩)
  AnkleDataStru solverMotorInput_;                          // 解算器电机输入数据(左,右电机位置)(左,右电机速度)(左,右电机扭矩)
  AnkleMotorToJointStatusStru solverStatus_;                // 解算器 motor to joint 的迭代状态
  AnkleDataStru solverJointOutput_;                         // 解算器脚踝关节输出数据 (roll,pitch位置)(roll,pitch速度)(roll,pitch力矩)
  AnkleDataStru realJointOutput_;                           // 原始脚踝关节输出数据(roll,pitch位置)(roll,pitch速度)(roll,pitch力矩)
};

struct AnkleJointToMotorDataStru
{
  AnkleDataStru realJointInput_;                            // 原始脚踝关节输入数据(roll,pitch位置)(roll,pitch速度)(roll,pitch力矩)
  AnkleDataStru solverJointInput_;                          // 解算器脚踝关节输入数据(roll,pitch位置)(roll,pitch速度)(roll,pitch力矩)
  AnkleDataStru solverMotorOutput_;                         // 解算器电机输出数据(左,右电机位置)(左,右电机速度)(左,右电机扭矩)
  AnkleDataStru realMotorOutput_;                           // 原始电机输出数据(左,右电机位置)(左,右电机速度)(左,右电机扭矩)
};

// 脚踝解算数据结构
struct AnkleSolveDebugDataStru
{
  // 当前控制循环的电机输入数据是否准备好
  bool isRealInputMotorDataReady_;
  // READ函数: motor to joint 结果（脚踝电机（左连杆电机，右连杆电机）信息转换为脚踝roll pitch信息）
  AnkleMotorToJointDataStru motorToJoint_;
  // WRITE函数: joint to motor 结果 （脚踝 roll pitch信息转换为脚踝电机（左连杆电机，右连杆电机）信息）
  AnkleJointToMotorDataStru jointToMotor_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 工具函数
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d getRotationMatrix(const double& theta, const std::string& axis);
Eigen::Matrix4d getTranslationMatrix(const double& x, const double& y, const double& z);
Eigen::Matrix4d getAnkleTransformationMatrix(const Eigen::Vector2d& ankle_pos,
                                             const std::string& ankle_config,
                                             const double& d_ankle);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HumanoidParallelAnkleSolver 类
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class HumanoidParallelAnkleSolver {
public:
  std::string motor_config_;  // 电机朝向配置 (X or x: X axis, Y or y: Y axis)
  std::string ankle_config_;  // 脚踝关节顺序配置 (RTP or rtp: 脚踝先pitch后roll, PTR or ptr: 先roll后pitch)

  // 脚踝关节零位时各关键点相较于踝关节原点的空间位置，本项目的坐标原点为脚踝关节的坐标原点，(1 对应 left，2 对应 right)
  Eigen::Vector3d r_A1_0_3d_;
  Eigen::Vector3d r_A2_0_3d_;
  Eigen::Vector3d r_B1_0_3d_;
  Eigen::Vector3d r_B2_0_3d_;
  Eigen::Vector3d r_C1_0_3d_;
  Eigen::Vector3d r_C2_0_3d_;
  double d_ankle_;  // 脚踝 roll 到 pitch 轴的直线距离 (m)

  // 脚踝关节零位时各连杆数据
  Eigen::Vector3d r_bar1_0_3d_;
  Eigen::Vector3d r_bar2_0_3d_;
  Eigen::Vector3d r_rod1_0_3d_;
  Eigen::Vector3d r_rod2_0_3d_;
  double bar1_len_;
  double bar2_len_;
  double rod1_len_;
  double rod2_len_;

  // 电机驱动杆的旋转轴方向
  Eigen::Vector3d motor1_rotation_axis_;  // s_11
  Eigen::Vector3d motor2_rotation_axis_;  // s_21

  // 正向运动学中牛顿-拉夫森迭代公式的初始猜测值
  Eigen::Vector2d fk_init_ankle_pos_;  // {ankle_pos_roll, ankle_pos_pitch}
  Eigen::Vector2d fk_prev_ankle_pos_;  // 上一次的脚踝位置 (roll, pitch)
  double fk_error_tolerance_;          // 迭代收敛容差
  int fk_max_iter_;                    // 最大迭代次数

  // Roll和Pitch的限制范围
  Eigen::Vector2d roll_range_;
  Eigen::Vector2d pitch_range_;

  // 电机角度限制
  Eigen::Vector2d motor1_range_;
  Eigen::Vector2d motor2_range_;

  // 数据偏置
  Eigen::Vector2d realToSolverMotorPosOffsets_;   // (左连杆电机位置偏置，右连杆电机位置偏置)此偏置将"实际电机位置"转换到"脚踝解算器电机位置"
  Eigen::Vector2d solverToRealJointPosOffsets_;   // （roll位置偏置，pitch位置偏置）此偏置将"脚踝解算器关节位置"转换到"实际脚踝关节位置"
  // 脚踝解算数据
  AnkleSolveDebugDataStru data_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Constructor (1 对应 left，2 对应 right)
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  HumanoidParallelAnkleSolver(const std::string& motor_config,
                              const std::string& ankle_config,
                              const Eigen::Vector3d& r_A1_0,    // (x, y, z) (m)
                              const Eigen::Vector3d& r_A2_0,    // (x, y, z) (m)
                              const Eigen::Vector3d& r_B1_0,    // (x, y, z) (m)
                              const Eigen::Vector3d& r_B2_0,    // (x, y, z) (m)
                              const Eigen::Vector3d& r_C1_0,    // (x, y, z) (m)
                              const Eigen::Vector3d& r_C2_0,    // (x, y, z) (m)
                              const double d_ankle,
                              const Eigen::Vector2d& roll_range,    // (min, max) (radians)
                              const Eigen::Vector2d& pitch_range,   // (min, max) (radians)
                              const Eigen::Vector2d& left_motor_range,  // (min, max) (radians)
                              const Eigen::Vector2d& right_motor_range); // (min, max) (radians)

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Methods
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief 清空脚踝解算数据
   */
  void clearAnkleSolverDebugData();

  /**
   * @brief 逆向运动学：已知踝关节角度求解电机角度
   * @param ankle_pos 踝关节角度 (ankle_pos_roll, ankle_pos_pitch)(radians)
   * @return 电机角度 (motor1_pos, motor2_pos)(radians)
   */
  Eigen::Vector2d inverseKinematics(const Eigen::Vector2d& ankle_pos);

  /**
   * @brief 逆向运动学已知踝关节角度求解雅可比矩阵
   * @param ankle_pos 踝关节角度 (ankle_pos_roll, ankle_pos_pitch)(radians)
   * @return 电机角度 (motor1_pos, motor2_pos)(radians), 雅可比矩阵
   */
  std::pair<Eigen::Vector2d, Eigen::MatrixXd> inverseKinematicsWithJacobian(const Eigen::Vector2d& ankle_pos);

  /**
   * @brief 正向运动学通过牛顿-拉夫森迭代公式：已知电机角度求解踝关节角度
   * @param motor_pos 电机角度 (motor1_pos, motor2_pos)(radians)
   * @return 踝关节角度 (ankle_pos_roll, ankle_pos_pitch)(radians)，迭代次数，是否成功计算
   */
  std::tuple<Eigen::Vector2d, int, bool, double> forwardKinematics(const Eigen::Vector2d& motor_pos);

  /**
   * @brief 电机数据 -> 脚踝关节数据
   * @param motor_pos 电机角度 (motor1_pos, motor2_pos)(radians)
   * @param motor_vel 电机速度 (motor1_vel, motor2_vel)(radians/s)
   * @param motor_torque 电机扭矩 (motor1_torque, motor2_torque)(N·m)
   * @return 脚踝关节角度 (ankle_pos_roll, ankle_pos_pitch)(radians),
   *          脚踝关节速度 (ankle_vel_roll, ankle_vel_pitch)(radians/s),
   *          脚踝关节扭矩 (ankle_torque_roll, ankle_torque_pitch)(N·m)
   */
  std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, int, bool, double> motorToJoint(const Eigen::Vector2d& motor_pos,
                                                                             const Eigen::Vector2d& motor_vel,
                                                                             const Eigen::Vector2d& motor_torque);

  /**
   * @brief 脚踝关节数据 -> 电机数据
   * @param ankle_pos 脚踝关节角度 (ankle_pos_roll, ankle_pos_pitch)(radians)
   * @param ankle_vel 脚踝关节速度 (ankle_vel_roll, ankle_vel_pitch)(radians/s)
   * @param ankle_torque 脚踝关节扭矩 (ankle_torque_roll, ankle_torque_pitch)(N·m)
   * @return 电机角度 (motor1_pos, motor2_pos)(radians),
   *          电机速度 (motor1_vel, motor2_vel)(radians/s),
   *          电机扭矩 (motor1_torque, motor2_torque)(N·m)
   */
  std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d> jointToMotor(const Eigen::Vector2d& ankle_pos,
                                                                             const Eigen::Vector2d& ankle_vel,
                                                                             const Eigen::Vector2d& ankle_torque);
};

#endif  // HUMANOID_PARALLEL_ANKLE_SOLVER_H_
