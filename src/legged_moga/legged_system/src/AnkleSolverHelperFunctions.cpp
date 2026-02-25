#include "AnkleSolver.h"
#include <vector>
#include <limits>

/*
 * 人形机器人踝关节解算器辅助函数
 */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 打印向量
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename VectorType>
void printVector(const Eigen::MatrixBase<VectorType>& vector) {
  std::cout << "[";
  for (int i = 0; i < vector.size(); i++) {
    std::cout << vector(i) << ((i == vector.size() - 1) ? "" : " ");
  }
  std::cout << "]" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 获取旋转矩阵
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d getRotationMatrix(const double& theta, const std::string& axis) {

  Eigen::Matrix4d rotation_matrix = Eigen::Matrix4d::Identity();

  if (axis == "x" || axis == "X") {
    rotation_matrix << 1, 0, 0, 0,
                       0, cos(theta), -sin(theta), 0,
                       0, sin(theta), cos(theta), 0,
                       0, 0, 0, 1;
  } else if (axis == "y" || axis == "Y") {
    rotation_matrix << cos(theta), 0, sin(theta), 0,
                       0, 1, 0, 0,
                       -sin(theta), 0, cos(theta), 0,
                       0, 0, 0, 1;
  } else if (axis == "z" || axis == "Z") {
    rotation_matrix << cos(theta), -sin(theta), 0, 0,
                       sin(theta), cos(theta), 0, 0,
                       0, 0, 1, 0,
                       0, 0, 0, 1;
  }
  return rotation_matrix;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 获取平移矩阵
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d getTranslationMatrix(const double& x, const double& y, const double& z) {
  Eigen::Matrix4d translation_matrix = Eigen::Matrix4d::Identity();
  translation_matrix << 1, 0, 0, x,
                        0, 1, 0, y,
                        0, 0, 1, z,
                        0, 0, 0, 1;
  return translation_matrix;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 获取踝关节齐次变换矩阵
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d getAnkleTransformationMatrix(const Eigen::Vector2d& ankle_pos,
                                            const std::string& ankle_config,
                                            const double& d_ankle) {
  Eigen::Matrix4d T_ankle = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d R_roll = getRotationMatrix(ankle_pos(0), "x");
  Eigen::Matrix4d R_pitch = getRotationMatrix(ankle_pos(1), "y");
  Eigen::Matrix4d T_d = getTranslationMatrix(0, 0, d_ankle);
  Eigen::Matrix4d T_d_neg = getTranslationMatrix(0, 0, -d_ankle);

  if (ankle_config == "RTP" || ankle_config == "rtp") {
    T_ankle = R_pitch * T_d_neg * R_roll * T_d;
  } else if (ankle_config == "PTR" || ankle_config == "ptr") {
    T_ankle = R_roll * T_d_neg * R_pitch * T_d;
  }
  return T_ankle;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 应用关节角度限制
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d applyJointLimit(const Eigen::Vector2d& joint_pos,
                                const Eigen::Vector2d& joint1_pos_range,
                                const Eigen::Vector2d& joint2_pos_range) {
// 创建局部变量存储关节角度
Eigen::Vector2d limited_joint_pos = joint_pos;

// 如果超过了关节限制，则使用关节的最大值
limited_joint_pos(0) = std::max(joint1_pos_range(0), std::min(limited_joint_pos(0), joint1_pos_range(1)));
limited_joint_pos(1) = std::max(joint2_pos_range(0), std::min(limited_joint_pos(1), joint2_pos_range(1)));

return limited_joint_pos;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 应用多边形限制
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 计算点到线段的最短距离和投影点
std::pair<double, Eigen::Vector2d> pointToLineSegmentDistance(const Eigen::Vector2d& point,
                                                             const Eigen::Vector2d& line_start,
                                                             const Eigen::Vector2d& line_end) {
  Eigen::Vector2d line_vec = line_end - line_start;
  Eigen::Vector2d point_vec = point - line_start;

  double line_length_sq = line_vec.squaredNorm();
  if (line_length_sq < 1e-10) {
    // 线段退化为点
    return std::make_pair((point - line_start).norm(), line_start);
  }

  double t = point_vec.dot(line_vec) / line_length_sq;
  t = std::max(0.0, std::min(1.0, t)); // 限制在[0,1]范围内

  Eigen::Vector2d projection = line_start + t * line_vec;
  double distance = (point - projection).norm();

  return std::make_pair(distance, projection);
}

// 使用射线法判断点是否在多边形内部
bool isPointInPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) {
  int n = polygon.size();
  bool inside = false;

  for (int i = 0, j = n - 1; i < n; j = i++) {
    double xi = polygon[i](0), yi = polygon[i](1);
    double xj = polygon[j](0), yj = polygon[j](1);

    if (((yi > point(1)) != (yj > point(1))) &&
        (point(0) < (xj - xi) * (point(1) - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }

  return inside;
}

Eigen::Vector2d applySixPointPolygonLimit(const Eigen::Vector2d& joint_pos) {
  // 定义六边形顶点（按逆时针顺序，单位：弧度）
  // 顶点坐标：(0°,25°), (-70°,-25°), (-70°,-55°), (0°,-75°), (70°,-55°), (70°,-25°)
  // 使用静态变量避免每次调用时重新创建多边形
  static const std::vector<Eigen::Vector2d> polygon = {
    Eigen::Vector2d(0.0, 10.0) * DEG_TO_RAD,
    Eigen::Vector2d(-45.0, -20.0) * DEG_TO_RAD,
    Eigen::Vector2d(-45.0, -30.0) * DEG_TO_RAD,
    Eigen::Vector2d(0.0, -50.0) * DEG_TO_RAD,
    Eigen::Vector2d(45.0, -30.0) * DEG_TO_RAD,
    Eigen::Vector2d(45.0, -20.0) * DEG_TO_RAD
  };

  // 检查点是否在多边形内部
  if (isPointInPolygon(joint_pos, polygon)) {
    return joint_pos;
  }

  // 如果点在多边形外部，找到最近的边界点
  double min_distance = std::numeric_limits<double>::max();
  Eigen::Vector2d closest_point = joint_pos;

  int n = polygon.size();
  for (int i = 0; i < n; i++) {
    int next_i = (i + 1) % n;

    auto result = pointToLineSegmentDistance(joint_pos, polygon[i], polygon[next_i]);
    double distance = result.first;
    Eigen::Vector2d projection = result.second;

    if (distance < min_distance) {
      min_distance = distance;
      closest_point = projection;
    }
  }

  return closest_point;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 获取电机角度
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double getMotorPosition(const std::string& motor_config,
                        const std::string& ankle_config,
                        const Eigen::Vector3d& r_Ai_0,
                        const Eigen::Vector3d& r_Bi_0,
                        const Eigen::Vector3d& r_Ci_0,
                        const double d_ankle,
                        const Eigen::Vector2d& ankle_pos,
                        const int motor_id) {

  double Ai_x0 = r_Ai_0[0];
  double Ai_y0 = r_Ai_0[1];
  double Ai_z0 = r_Ai_0[2];

  double Bi_x0 = r_Bi_0[0];
  double Bi_y0 = r_Bi_0[1];
  double Bi_z0 = r_Bi_0[2];

  double Ci_x0 = r_Ci_0[0];
  double Ci_y0 = r_Ci_0[1];
  double Ci_z0 = r_Ci_0[2];

  double Ai_x0_2 = pow(Ai_x0, 2);
  double Ai_y0_2 = pow(Ai_y0, 2);
  double Ai_z0_2 = pow(Ai_z0, 2);

  double Bi_x0_2 = pow(Bi_x0, 2);
  double Bi_y0_2 = pow(Bi_y0, 2);
  double Bi_z0_2 = pow(Bi_z0, 2);

  double Ci_x0_2 = pow(Ci_x0, 2);
  double Ci_y0_2 = pow(Ci_y0, 2);
  double Ci_z0_2 = pow(Ci_z0, 2);

  double d_ankle_2 = pow(d_ankle, 2);

  double l_rod_i_square = pow(Ci_x0 - Bi_x0, 2) + pow(Ci_y0 - Bi_y0, 2) + pow(Ci_z0 - Bi_z0, 2);

  double sin_q_roll = sin(ankle_pos(0));
  double cos_q_roll = cos(ankle_pos(0));
  double sin_q_pitch = sin(ankle_pos(1));
  double cos_q_pitch = cos(ankle_pos(1));

  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  double theta = 0.0;

  if (motor_config == "X" || motor_config == "x") {
    if (ankle_config == "RTP" || ankle_config == "rtp") {
      a = -2*Ai_y0_2\
        +2*Ai_y0*Bi_y0\
        +2*Ai_y0*Ci_y0*cos_q_roll\
        -2*Ai_y0*Ci_z0*sin_q_roll\
        -2*Ai_y0*d_ankle*sin_q_roll\
        -2*Ai_z0_2\
        +2*Ai_z0*Bi_z0\
        -2*Ai_z0*Ci_x0*sin_q_pitch\
        +2*Ai_z0*Ci_y0*sin_q_roll*cos_q_pitch\
        +2*Ai_z0*Ci_z0*cos_q_pitch*cos_q_roll\
        +2*Ai_z0*d_ankle*cos_q_pitch*cos_q_roll\
        -2*Ai_z0*d_ankle*cos_q_pitch\
        -2*Bi_y0*Ci_y0*cos_q_roll\
        +2*Bi_y0*Ci_z0*sin_q_roll\
        +2*Bi_y0*d_ankle*sin_q_roll\
        +2*Bi_z0*Ci_x0*sin_q_pitch\
        -2*Bi_z0*Ci_y0*sin_q_roll*cos_q_pitch\
        -2*Bi_z0*Ci_z0*cos_q_pitch*cos_q_roll\
        -2*Bi_z0*d_ankle*cos_q_pitch*cos_q_roll\
        +2*Bi_z0*d_ankle*cos_q_pitch;

      b = -2*Ai_y0*Bi_z0\
        -2*Ai_y0*Ci_x0*sin_q_pitch\
        +2*Ai_y0*Ci_y0*sin_q_roll*cos_q_pitch\
        +2*Ai_y0*Ci_z0*cos_q_pitch*cos_q_roll\
        +2*Ai_y0*d_ankle*cos_q_pitch*cos_q_roll\
        -2*Ai_y0*d_ankle*cos_q_pitch\
        +2*Ai_z0*Bi_y0\
        -2*Ai_z0*Ci_y0*cos_q_roll\
        +2*Ai_z0*Ci_z0*sin_q_roll\
        +2*Ai_z0*d_ankle*sin_q_roll\
        +2*Bi_y0*Ci_x0*sin_q_pitch\
        -2*Bi_y0*Ci_y0*sin_q_roll*cos_q_pitch\
        -2*Bi_y0*Ci_z0*cos_q_pitch*cos_q_roll\
        -2*Bi_y0*d_ankle*cos_q_pitch*cos_q_roll\
        +2*Bi_y0*d_ankle*cos_q_pitch\
        +2*Bi_z0*Ci_y0*cos_q_roll\
        -2*Bi_z0*Ci_z0*sin_q_roll\
        -2*Bi_z0*d_ankle*sin_q_roll;

      c = -2*Ai_y0_2\
        +2*Ai_y0*Bi_y0\
        +2*Ai_y0*Ci_y0*cos_q_roll\
        -2*Ai_y0*Ci_z0*sin_q_roll\
        -2*Ai_y0*d_ankle*sin_q_roll\
        -2*Ai_z0_2\
        +2*Ai_z0*Bi_z0\
        -2*Ai_z0*Ci_x0*sin_q_pitch\
        +2*Ai_z0*Ci_y0*sin_q_roll*cos_q_pitch\
        +2*Ai_z0*Ci_z0*cos_q_pitch*cos_q_roll\
        +2*Ai_z0*d_ankle*cos_q_pitch*cos_q_roll\
        -2*Ai_z0*d_ankle*cos_q_pitch\
        -Bi_x0_2\
        +2*Bi_x0*Ci_x0*cos_q_pitch\
        +2*Bi_x0*Ci_y0*sin_q_pitch*sin_q_roll\
        +2*Bi_x0*Ci_z0*sin_q_pitch*cos_q_roll\
        +2*Bi_x0*d_ankle*sin_q_pitch*cos_q_roll\
        -2*Bi_x0*d_ankle*sin_q_pitch\
        -Bi_y0_2\
        -Bi_z0_2\
        -Ci_x0_2\
        -Ci_y0_2\
        +2*Ci_y0*d_ankle*sin_q_roll\
        -Ci_z0_2\
        +2*Ci_z0*d_ankle*cos_q_roll\
        -2*Ci_z0*d_ankle\
        +2*d_ankle_2*cos_q_roll\
        -2*d_ankle_2\
        +l_rod_i_square;

      if (motor_id == 1) {
        theta = asin((b * c + sqrt(pow(b,2) * pow(c,2) - (pow(a,2) + pow(b,2)) * (pow(c,2) - pow(a,2))))/(pow(a,2) + pow(b,2)));
      } else {
        theta = asin((b * c - sqrt(pow(b,2) * pow(c,2) - (pow(a,2) + pow(b,2)) * (pow(c,2) - pow(a,2))))/(pow(a,2) + pow(b,2)));
      }
    } else if (ankle_config == "PTR" || ankle_config == "ptr") {
      // TODO: Y轴朝向电机,脚踝先Roll,再平移,后Pitch
      a = 0;

      b = 0;

      c = 0;

      theta = 0;
    }
  } else if (motor_config == "Y" || motor_config == "y") {
    if (ankle_config == "RTP" || ankle_config == "rtp") {
      a = -2*Ai_x0_2\
        +2*Ai_x0*Bi_x0\
        +2*Ai_x0*Ci_x0*cos_q_pitch\
        +2*Ai_x0*Ci_y0*sin_q_pitch*sin_q_roll\
        +2*Ai_x0*Ci_z0*sin_q_pitch*cos_q_roll\
        +2*Ai_x0*d_ankle*sin_q_pitch*cos_q_roll\
        -2*Ai_x0*d_ankle*sin_q_pitch\
        -2*Ai_z0_2\
        +2*Ai_z0*Bi_z0\
        -2*Ai_z0*Ci_x0*sin_q_pitch\
        +2*Ai_z0*Ci_y0*sin_q_roll*cos_q_pitch\
        +2*Ai_z0*Ci_z0*cos_q_pitch*cos_q_roll\
        +2*Ai_z0*d_ankle*cos_q_pitch*cos_q_roll\
        -2*Ai_z0*d_ankle*cos_q_pitch\
        -2*Bi_x0*Ci_x0*cos_q_pitch\
        -2*Bi_x0*Ci_y0*sin_q_pitch*sin_q_roll\
        -2*Bi_x0*Ci_z0*sin_q_pitch*cos_q_roll\
        -2*Bi_x0*d_ankle*sin_q_pitch*cos_q_roll\
        +2*Bi_x0*d_ankle*sin_q_pitch\
        +2*Bi_z0*Ci_x0*sin_q_pitch\
        -2*Bi_z0*Ci_y0*sin_q_roll*cos_q_pitch\
        -2*Bi_z0*Ci_z0*cos_q_pitch*cos_q_roll\
        -2*Bi_z0*d_ankle*cos_q_pitch*cos_q_roll\
        +2*Bi_z0*d_ankle*cos_q_pitch;

      b = +2*Ai_x0*Bi_z0\
        +2*Ai_x0*Ci_x0*sin_q_pitch\
        -2*Ai_x0*Ci_y0*sin_q_roll*cos_q_pitch\
        -2*Ai_x0*Ci_z0*cos_q_pitch*cos_q_roll\
        -2*Ai_x0*d_ankle*cos_q_pitch*cos_q_roll\
        +2*Ai_x0*d_ankle*cos_q_pitch\
        -2*Ai_z0*Bi_x0\
        +2*Ai_z0*Ci_x0*cos_q_pitch\
        +2*Ai_z0*Ci_y0*sin_q_pitch*sin_q_roll\
        +2*Ai_z0*Ci_z0*sin_q_pitch*cos_q_roll\
        +2*Ai_z0*d_ankle*sin_q_pitch*cos_q_roll\
        -2*Ai_z0*d_ankle*sin_q_pitch\
        -2*Bi_x0*Ci_x0*sin_q_pitch\
        +2*Bi_x0*Ci_y0*sin_q_roll*cos_q_pitch\
        +2*Bi_x0*Ci_z0*cos_q_pitch*cos_q_roll\
        +2*Bi_x0*d_ankle*cos_q_pitch*cos_q_roll\
        -2*Bi_x0*d_ankle*cos_q_pitch\
        -2*Bi_z0*Ci_x0*cos_q_pitch\
        -2*Bi_z0*Ci_y0*sin_q_pitch*sin_q_roll\
        -2*Bi_z0*Ci_z0*sin_q_pitch*cos_q_roll\
        -2*Bi_z0*d_ankle*sin_q_pitch*cos_q_roll\
        +2*Bi_z0*d_ankle*sin_q_pitch;

      c = -2*Ai_x0_2\
        +2*Ai_x0*Bi_x0\
        +2*Ai_x0*Ci_x0*cos_q_pitch\
        +2*Ai_x0*Ci_y0*sin_q_pitch*sin_q_roll\
        +2*Ai_x0*Ci_z0*sin_q_pitch*cos_q_roll\
        +2*Ai_x0*d_ankle*sin_q_pitch*cos_q_roll\
        -2*Ai_x0*d_ankle*sin_q_pitch\
        -2*Ai_z0_2\
        +2*Ai_z0*Bi_z0\
        -2*Ai_z0*Ci_x0*sin_q_pitch\
        +2*Ai_z0*Ci_y0*sin_q_roll*cos_q_pitch\
        +2*Ai_z0*Ci_z0*cos_q_pitch*cos_q_roll\
        +2*Ai_z0*d_ankle*cos_q_pitch*cos_q_roll\
        -2*Ai_z0*d_ankle*cos_q_pitch\
        -Bi_x0_2\
        -Bi_y0_2\
        +2*Bi_y0*Ci_y0*cos_q_roll\
        -2*Bi_y0*Ci_z0*sin_q_roll\
        -2*Bi_y0*d_ankle*sin_q_roll\
        -Bi_z0_2\
        -Ci_x0_2\
        -Ci_y0_2\
        +2*Ci_y0*d_ankle*sin_q_roll\
        -Ci_z0_2\
        +2*Ci_z0*d_ankle*cos_q_roll\
        -2*Ci_z0*d_ankle\
        +2*d_ankle_2*cos_q_roll\
        -2*d_ankle_2\
        +l_rod_i_square;

      theta = asin((b * c + sqrt(pow(b,2) * pow(c,2) - (pow(a,2) + pow(b,2)) * (pow(c,2) - pow(a,2))))/(pow(a,2) + pow(b,2)));

    } else if (ankle_config == "PTR" || ankle_config == "ptr") {
      // TODO: Y轴朝向电机,脚踝先Roll,再平移,后Pitch
      a = 0;

      b = 0;

      c = 0;

      theta = 0;
    }
  }
  return theta;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 获取雅可比矩阵
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 获取雅可比矩阵ab项 ////////////////////////////////////////////////////////////////////////////////////////////////////
double getJacobianABTerm(
    const std::string& motor_config,
    const std::string& ankle_config,
    const Eigen::Vector3d& r_Ai_0,
    const Eigen::Vector3d& r_Bi_0,
    const Eigen::Vector3d& r_Ci_0,
    const double d_ankle,
    const double d_ankle_2,
    const double sin_q_roll,
    const double cos_q_roll,
    const double sin_q_pitch,
    const double cos_q_pitch,
    const double theta) {
  double Ai_x0 = r_Ai_0(0);
  double Ai_y0 = r_Ai_0(1);
  double Ai_z0 = r_Ai_0(2);

  double Bi_x0 = r_Bi_0(0);
  double Bi_y0 = r_Bi_0(1);
  double Bi_z0 = r_Bi_0(2);

  double Ci_x0 = r_Ci_0(0);
  double Ci_y0 = r_Ci_0(1);
  double Ci_z0 = r_Ci_0(2);

  double Ai_x0_2 = pow(Ai_x0, 2);
  double Ai_y0_2 = pow(Ai_y0, 2);
  double Ai_z0_2 = pow(Ai_z0, 2);

  double Bi_x0_2 = pow(Bi_x0, 2);
  double Bi_y0_2 = pow(Bi_y0, 2);
  double Bi_z0_2 = pow(Bi_z0, 2);

  double Ci_x0_2 = pow(Ci_x0, 2);
  double Ci_y0_2 = pow(Ci_y0, 2);
  double Ci_z0_2 = pow(Ci_z0, 2);

  double sin_theta = sin(theta);
  double cos_theta = cos(theta);

  double ab_term = 0.0;

  if (motor_config == "Y" || motor_config == "y") {
    if (ankle_config == "RTP" || ankle_config == "rtp") {
      ab_term = +2*Ai_x0_2*sin_theta\
        -2*Ai_x0*Bi_x0*sin_theta\
        +2*Ai_x0*Bi_z0*cos_theta\
        +2*Ai_x0*Ci_x0*sin_q_pitch*cos_theta\
        -2*Ai_x0*Ci_x0*sin_theta*cos_q_pitch\
        -2*Ai_x0*Ci_y0*sin_q_pitch*sin_q_roll*sin_theta\
        -2*Ai_x0*Ci_y0*sin_q_roll*cos_q_pitch*cos_theta\
        -2*Ai_x0*Ci_z0*sin_q_pitch*sin_theta*cos_q_roll\
        -2*Ai_x0*Ci_z0*cos_q_pitch*cos_q_roll*cos_theta\
        -2*Ai_x0*d_ankle*sin_q_pitch*sin_theta*cos_q_roll\
        +2*Ai_x0*d_ankle*sin_q_pitch*sin_theta\
        -2*Ai_x0*d_ankle*cos_q_pitch*cos_q_roll*cos_theta\
        +2*Ai_x0*d_ankle*cos_q_pitch*cos_theta\
        +2*Ai_z0_2*sin_theta\
        -2*Ai_z0*Bi_x0*cos_theta\
        -2*Ai_z0*Bi_z0*sin_theta\
        +2*Ai_z0*Ci_x0*sin_q_pitch*sin_theta\
        +2*Ai_z0*Ci_x0*cos_q_pitch*cos_theta\
        +2*Ai_z0*Ci_y0*sin_q_pitch*sin_q_roll*cos_theta\
        -2*Ai_z0*Ci_y0*sin_q_roll*sin_theta*cos_q_pitch\
        +2*Ai_z0*Ci_z0*sin_q_pitch*cos_q_roll*cos_theta\
        -2*Ai_z0*Ci_z0*sin_theta*cos_q_pitch*cos_q_roll\
        +2*Ai_z0*d_ankle*sin_q_pitch*cos_q_roll*cos_theta\
        -2*Ai_z0*d_ankle*sin_q_pitch*cos_theta\
        -2*Ai_z0*d_ankle*sin_theta*cos_q_pitch*cos_q_roll\
        +2*Ai_z0*d_ankle*sin_theta*cos_q_pitch\
        -2*Bi_x0*Ci_x0*sin_q_pitch*cos_theta\
        +2*Bi_x0*Ci_x0*sin_theta*cos_q_pitch\
        +2*Bi_x0*Ci_y0*sin_q_pitch*sin_q_roll*sin_theta\
        +2*Bi_x0*Ci_y0*sin_q_roll*cos_q_pitch*cos_theta\
        +2*Bi_x0*Ci_z0*sin_q_pitch*sin_theta*cos_q_roll\
        +2*Bi_x0*Ci_z0*cos_q_pitch*cos_q_roll*cos_theta\
        +2*Bi_x0*d_ankle*sin_q_pitch*sin_theta*cos_q_roll\
        -2*Bi_x0*d_ankle*sin_q_pitch*sin_theta\
        +2*Bi_x0*d_ankle*cos_q_pitch*cos_q_roll*cos_theta\
        -2*Bi_x0*d_ankle*cos_q_pitch*cos_theta\
        -2*Bi_z0*Ci_x0*sin_q_pitch*sin_theta\
        -2*Bi_z0*Ci_x0*cos_q_pitch*cos_theta\
        -2*Bi_z0*Ci_y0*sin_q_pitch*sin_q_roll*cos_theta\
        +2*Bi_z0*Ci_y0*sin_q_roll*sin_theta*cos_q_pitch\
        -2*Bi_z0*Ci_z0*sin_q_pitch*cos_q_roll*cos_theta\
        +2*Bi_z0*Ci_z0*sin_theta*cos_q_pitch*cos_q_roll\
        -2*Bi_z0*d_ankle*sin_q_pitch*cos_q_roll*cos_theta\
        +2*Bi_z0*d_ankle*sin_q_pitch*cos_theta\
        +2*Bi_z0*d_ankle*sin_theta*cos_q_pitch*cos_q_roll\
        -2*Bi_z0*d_ankle*sin_theta*cos_q_pitch;

    } else if (ankle_config == "PTR" || ankle_config == "ptr") {
      ab_term = 0.0;

    }

  } else if (motor_config == "X" || motor_config == "x") {
    if (ankle_config == "RTP" || ankle_config == "rtp") {
      ab_term = +2*Ai_y0_2*sin_theta\
        -2*Ai_y0*Bi_y0*sin_theta\
        -2*Ai_y0*Bi_z0*cos_theta\
        -2*Ai_y0*Ci_x0*sin_q_pitch*cos_theta\
        +2*Ai_y0*Ci_y0*sin_q_roll*cos_q_pitch*cos_theta\
        -2*Ai_y0*Ci_y0*sin_theta*cos_q_roll\
        +2*Ai_y0*Ci_z0*sin_q_roll*sin_theta\
        +2*Ai_y0*Ci_z0*cos_q_pitch*cos_q_roll*cos_theta\
        +2*Ai_y0*d_ankle*sin_q_roll*sin_theta\
        +2*Ai_y0*d_ankle*cos_q_pitch*cos_q_roll*cos_theta\
        -2*Ai_y0*d_ankle*cos_q_pitch*cos_theta\
        +2*Ai_z0_2*sin_theta\
        +2*Ai_z0*Bi_y0*cos_theta\
        -2*Ai_z0*Bi_z0*sin_theta\
        +2*Ai_z0*Ci_x0*sin_q_pitch*sin_theta\
        -2*Ai_z0*Ci_y0*sin_q_roll*sin_theta*cos_q_pitch\
        -2*Ai_z0*Ci_y0*cos_q_roll*cos_theta\
        +2*Ai_z0*Ci_z0*sin_q_roll*cos_theta\
        -2*Ai_z0*Ci_z0*sin_theta*cos_q_pitch*cos_q_roll\
        +2*Ai_z0*d_ankle*sin_q_roll*cos_theta\
        -2*Ai_z0*d_ankle*sin_theta*cos_q_pitch*cos_q_roll\
        +2*Ai_z0*d_ankle*sin_theta*cos_q_pitch\
        +2*Bi_y0*Ci_x0*sin_q_pitch*cos_theta\
        -2*Bi_y0*Ci_y0*sin_q_roll*cos_q_pitch*cos_theta\
        +2*Bi_y0*Ci_y0*sin_theta*cos_q_roll\
        -2*Bi_y0*Ci_z0*sin_q_roll*sin_theta\
        -2*Bi_y0*Ci_z0*cos_q_pitch*cos_q_roll*cos_theta\
        -2*Bi_y0*d_ankle*sin_q_roll*sin_theta\
        -2*Bi_y0*d_ankle*cos_q_pitch*cos_q_roll*cos_theta\
        +2*Bi_y0*d_ankle*cos_q_pitch*cos_theta\
        -2*Bi_z0*Ci_x0*sin_q_pitch*sin_theta\
        +2*Bi_z0*Ci_y0*sin_q_roll*sin_theta*cos_q_pitch\
        +2*Bi_z0*Ci_y0*cos_q_roll*cos_theta\
        -2*Bi_z0*Ci_z0*sin_q_roll*cos_theta\
        +2*Bi_z0*Ci_z0*sin_theta*cos_q_pitch*cos_q_roll\
        -2*Bi_z0*d_ankle*sin_q_roll*cos_theta\
        +2*Bi_z0*d_ankle*sin_theta*cos_q_pitch*cos_q_roll\
        -2*Bi_z0*d_ankle*sin_theta*cos_q_pitch;

    } else if (ankle_config == "PTR" || ankle_config == "ptr") {
      ab_term = 0.0;

    }
  }
  return ab_term;
}

// 获取雅可比矩阵c项 //////////////////////////////////////////////////////////////////////////////////////////////////////
double getJacobianCTerm(
    const std::string& motor_config,
    const std::string& ankle_config,
    const Eigen::Vector3d& r_Ai_0,
    const Eigen::Vector3d& r_Bi_0,
    const Eigen::Vector3d& r_Ci_0,
    const double d_ankle,
    const double d_ankle_2,
    const double sin_q_roll,
    const double cos_q_roll,
    const double sin_q_pitch,
    const double cos_q_pitch,
    const double theta) {
  double Ai_x0 = r_Ai_0(0);
  double Ai_y0 = r_Ai_0(1);
  double Ai_z0 = r_Ai_0(2);

  double Bi_x0 = r_Bi_0(0);
  double Bi_y0 = r_Bi_0(1);
  double Bi_z0 = r_Bi_0(2);

  double Ci_x0 = r_Ci_0(0);
  double Ci_y0 = r_Ci_0(1);
  double Ci_z0 = r_Ci_0(2);

  double Ai_x0_2 = pow(Ai_x0, 2);
  double Ai_y0_2 = pow(Ai_y0, 2);
  double Ai_z0_2 = pow(Ai_z0, 2);

  double Bi_x0_2 = pow(Bi_x0, 2);
  double Bi_y0_2 = pow(Bi_y0, 2);
  double Bi_z0_2 = pow(Bi_z0, 2);

  double Ci_x0_2 = pow(Ci_x0, 2);
  double Ci_y0_2 = pow(Ci_y0, 2);
  double Ci_z0_2 = pow(Ci_z0, 2);

  double sin_theta = sin(theta);
  double cos_theta = cos(theta);

  double c_term = 0.0;

  if (motor_config == "Y" || motor_config == "y") {
    if (ankle_config == "RTP" || ankle_config == "rtp") {
      c_term = +2*Ai_x0*Ci_y0*sin_q_pitch*cos_q_roll*cos_theta\
        -2*Ai_x0*Ci_y0*sin_q_pitch*cos_q_roll\
        -2*Ai_x0*Ci_y0*sin_theta*cos_q_pitch*cos_q_roll\
        -2*Ai_x0*Ci_z0*sin_q_pitch*sin_q_roll*cos_theta\
        +2*Ai_x0*Ci_z0*sin_q_pitch*sin_q_roll\
        +2*Ai_x0*Ci_z0*sin_q_roll*sin_theta*cos_q_pitch\
        -2*Ai_x0*d_ankle*sin_q_pitch*sin_q_roll*cos_theta\
        +2*Ai_x0*d_ankle*sin_q_pitch*sin_q_roll\
        +2*Ai_x0*d_ankle*sin_q_roll*sin_theta*cos_q_pitch\
        +2*Ai_z0*Ci_y0*sin_q_pitch*sin_theta*cos_q_roll\
        +2*Ai_z0*Ci_y0*cos_q_pitch*cos_q_roll*cos_theta\
        -2*Ai_z0*Ci_y0*cos_q_pitch*cos_q_roll\
        -2*Ai_z0*Ci_z0*sin_q_pitch*sin_q_roll*sin_theta\
        -2*Ai_z0*Ci_z0*sin_q_roll*cos_q_pitch*cos_theta\
        +2*Ai_z0*Ci_z0*sin_q_roll*cos_q_pitch\
        -2*Ai_z0*d_ankle*sin_q_pitch*sin_q_roll*sin_theta\
        -2*Ai_z0*d_ankle*sin_q_roll*cos_q_pitch*cos_theta\
        +2*Ai_z0*d_ankle*sin_q_roll*cos_q_pitch\
        -2*Bi_x0*Ci_y0*sin_q_pitch*cos_q_roll*cos_theta\
        +2*Bi_x0*Ci_y0*sin_theta*cos_q_pitch*cos_q_roll\
        +2*Bi_x0*Ci_z0*sin_q_pitch*sin_q_roll*cos_theta\
        -2*Bi_x0*Ci_z0*sin_q_roll*sin_theta*cos_q_pitch\
        +2*Bi_x0*d_ankle*sin_q_pitch*sin_q_roll*cos_theta\
        -2*Bi_x0*d_ankle*sin_q_roll*sin_theta*cos_q_pitch\
        +2*Bi_y0*Ci_y0*sin_q_roll\
        +2*Bi_y0*Ci_z0*cos_q_roll\
        +2*Bi_y0*d_ankle*cos_q_roll\
        -2*Bi_z0*Ci_y0*sin_q_pitch*sin_theta*cos_q_roll\
        -2*Bi_z0*Ci_y0*cos_q_pitch*cos_q_roll*cos_theta\
        +2*Bi_z0*Ci_z0*sin_q_pitch*sin_q_roll*sin_theta\
        +2*Bi_z0*Ci_z0*sin_q_roll*cos_q_pitch*cos_theta\
        +2*Bi_z0*d_ankle*sin_q_pitch*sin_q_roll*sin_theta\
        +2*Bi_z0*d_ankle*sin_q_roll*cos_q_pitch*cos_theta\
        -2*Ci_y0*d_ankle*cos_q_roll\
        +2*Ci_z0*d_ankle*sin_q_roll\
        +2*d_ankle_2*sin_q_roll;

    } else if (ankle_config == "PTR" || ankle_config == "ptr") {
      c_term = 0.0;

    }

  } else if (motor_config == "X" || motor_config == "x") {
    if (ankle_config == "RTP" || ankle_config == "rtp") {
      c_term = -2*Ai_y0*Ci_y0*sin_q_roll*cos_theta\
        +2*Ai_y0*Ci_y0*sin_q_roll\
        +2*Ai_y0*Ci_y0*sin_theta*cos_q_pitch*cos_q_roll\
        -2*Ai_y0*Ci_z0*sin_q_roll*sin_theta*cos_q_pitch\
        -2*Ai_y0*Ci_z0*cos_q_roll*cos_theta\
        +2*Ai_y0*Ci_z0*cos_q_roll\
        -2*Ai_y0*d_ankle*sin_q_roll*sin_theta*cos_q_pitch\
        -2*Ai_y0*d_ankle*cos_q_roll*cos_theta\
        +2*Ai_y0*d_ankle*cos_q_roll\
        +2*Ai_z0*Ci_y0*sin_q_roll*sin_theta\
        +2*Ai_z0*Ci_y0*cos_q_pitch*cos_q_roll*cos_theta\
        -2*Ai_z0*Ci_y0*cos_q_pitch*cos_q_roll\
        -2*Ai_z0*Ci_z0*sin_q_roll*cos_q_pitch*cos_theta\
        +2*Ai_z0*Ci_z0*sin_q_roll*cos_q_pitch\
        +2*Ai_z0*Ci_z0*sin_theta*cos_q_roll\
        -2*Ai_z0*d_ankle*sin_q_roll*cos_q_pitch*cos_theta\
        +2*Ai_z0*d_ankle*sin_q_roll*cos_q_pitch\
        +2*Ai_z0*d_ankle*sin_theta*cos_q_roll\
        -2*Bi_x0*Ci_y0*sin_q_pitch*cos_q_roll\
        +2*Bi_x0*Ci_z0*sin_q_pitch*sin_q_roll\
        +2*Bi_x0*d_ankle*sin_q_pitch*sin_q_roll\
        +2*Bi_y0*Ci_y0*sin_q_roll*cos_theta\
        -2*Bi_y0*Ci_y0*sin_theta*cos_q_pitch*cos_q_roll\
        +2*Bi_y0*Ci_z0*sin_q_roll*sin_theta*cos_q_pitch\
        +2*Bi_y0*Ci_z0*cos_q_roll*cos_theta\
        +2*Bi_y0*d_ankle*sin_q_roll*sin_theta*cos_q_pitch\
        +2*Bi_y0*d_ankle*cos_q_roll*cos_theta\
        -2*Bi_z0*Ci_y0*sin_q_roll*sin_theta\
        -2*Bi_z0*Ci_y0*cos_q_pitch*cos_q_roll*cos_theta\
        +2*Bi_z0*Ci_z0*sin_q_roll*cos_q_pitch*cos_theta\
        -2*Bi_z0*Ci_z0*sin_theta*cos_q_roll\
        +2*Bi_z0*d_ankle*sin_q_roll*cos_q_pitch*cos_theta\
        -2*Bi_z0*d_ankle*sin_theta*cos_q_roll\
        -2*Ci_y0*d_ankle*cos_q_roll\
        +2*Ci_z0*d_ankle*sin_q_roll\
        +2*d_ankle_2*sin_q_roll;

    } else if (ankle_config == "PTR" || ankle_config == "ptr") {
      c_term = 0.0;

    }
  }
  return c_term;
}

// 获取雅可比矩阵d项 //////////////////////////////////////////////////////////////////////////////////////////////////////
double getJacobianDTerm(
    const std::string& motor_config,
    const std::string& ankle_config,
    const Eigen::Vector3d& r_Ai_0,
    const Eigen::Vector3d& r_Bi_0,
    const Eigen::Vector3d& r_Ci_0,
    const double d_ankle,
    const double d_ankle_2,
    const double sin_q_roll,
    const double cos_q_roll,
    const double sin_q_pitch,
    const double cos_q_pitch,
    const double theta) {
  double Ai_x0 = r_Ai_0(0);
  double Ai_y0 = r_Ai_0(1);
  double Ai_z0 = r_Ai_0(2);

  double Bi_x0 = r_Bi_0(0);
  double Bi_y0 = r_Bi_0(1);
  double Bi_z0 = r_Bi_0(2);

  double Ci_x0 = r_Ci_0(0);
  double Ci_y0 = r_Ci_0(1);
  double Ci_z0 = r_Ci_0(2);

  double Ai_x0_2 = pow(Ai_x0, 2);
  double Ai_y0_2 = pow(Ai_y0, 2);
  double Ai_z0_2 = pow(Ai_z0, 2);

  double Bi_x0_2 = pow(Bi_x0, 2);
  double Bi_y0_2 = pow(Bi_y0, 2);
  double Bi_z0_2 = pow(Bi_z0, 2);

  double Ci_x0_2 = pow(Ci_x0, 2);
  double Ci_y0_2 = pow(Ci_y0, 2);
  double Ci_z0_2 = pow(Ci_z0, 2);

  double sin_theta = sin(theta);
  double cos_theta = cos(theta);

  double d_term = 0.0;

  if (motor_config == "Y" || motor_config == "y") {
    if (ankle_config == "RTP" || ankle_config == "rtp") {
      d_term = -2*Ai_x0*Ci_x0*sin_q_pitch*cos_theta\
        +2*Ai_x0*Ci_x0*sin_q_pitch\
        +2*Ai_x0*Ci_x0*sin_theta*cos_q_pitch\
        +2*Ai_x0*Ci_y0*sin_q_pitch*sin_q_roll*sin_theta\
        +2*Ai_x0*Ci_y0*sin_q_roll*cos_q_pitch*cos_theta\
        -2*Ai_x0*Ci_y0*sin_q_roll*cos_q_pitch\
        +2*Ai_x0*Ci_z0*sin_q_pitch*sin_theta*cos_q_roll\
        +2*Ai_x0*Ci_z0*cos_q_pitch*cos_q_roll*cos_theta\
        -2*Ai_x0*Ci_z0*cos_q_pitch*cos_q_roll\
        +2*Ai_x0*d_ankle*sin_q_pitch*sin_theta*cos_q_roll\
        -2*Ai_x0*d_ankle*sin_q_pitch*sin_theta\
        +2*Ai_x0*d_ankle*cos_q_pitch*cos_q_roll*cos_theta\
        -2*Ai_x0*d_ankle*cos_q_pitch*cos_q_roll\
        -2*Ai_x0*d_ankle*cos_q_pitch*cos_theta\
        +2*Ai_x0*d_ankle*cos_q_pitch\
        -2*Ai_z0*Ci_x0*sin_q_pitch*sin_theta\
        -2*Ai_z0*Ci_x0*cos_q_pitch*cos_theta\
        +2*Ai_z0*Ci_x0*cos_q_pitch\
        -2*Ai_z0*Ci_y0*sin_q_pitch*sin_q_roll*cos_theta\
        +2*Ai_z0*Ci_y0*sin_q_pitch*sin_q_roll\
        +2*Ai_z0*Ci_y0*sin_q_roll*sin_theta*cos_q_pitch\
        -2*Ai_z0*Ci_z0*sin_q_pitch*cos_q_roll*cos_theta\
        +2*Ai_z0*Ci_z0*sin_q_pitch*cos_q_roll\
        +2*Ai_z0*Ci_z0*sin_theta*cos_q_pitch*cos_q_roll\
        -2*Ai_z0*d_ankle*sin_q_pitch*cos_q_roll*cos_theta\
        +2*Ai_z0*d_ankle*sin_q_pitch*cos_q_roll\
        +2*Ai_z0*d_ankle*sin_q_pitch*cos_theta\
        -2*Ai_z0*d_ankle*sin_q_pitch\
        +2*Ai_z0*d_ankle*sin_theta*cos_q_pitch*cos_q_roll\
        -2*Ai_z0*d_ankle*sin_theta*cos_q_pitch\
        +2*Bi_x0*Ci_x0*sin_q_pitch*cos_theta\
        -2*Bi_x0*Ci_x0*sin_theta*cos_q_pitch\
        -2*Bi_x0*Ci_y0*sin_q_pitch*sin_q_roll*sin_theta\
        -2*Bi_x0*Ci_y0*sin_q_roll*cos_q_pitch*cos_theta\
        -2*Bi_x0*Ci_z0*sin_q_pitch*sin_theta*cos_q_roll\
        -2*Bi_x0*Ci_z0*cos_q_pitch*cos_q_roll*cos_theta\
        -2*Bi_x0*d_ankle*sin_q_pitch*sin_theta*cos_q_roll\
        +2*Bi_x0*d_ankle*sin_q_pitch*sin_theta\
        -2*Bi_x0*d_ankle*cos_q_pitch*cos_q_roll*cos_theta\
        +2*Bi_x0*d_ankle*cos_q_pitch*cos_theta\
        +2*Bi_z0*Ci_x0*sin_q_pitch*sin_theta\
        +2*Bi_z0*Ci_x0*cos_q_pitch*cos_theta\
        +2*Bi_z0*Ci_y0*sin_q_pitch*sin_q_roll*cos_theta\
        -2*Bi_z0*Ci_y0*sin_q_roll*sin_theta*cos_q_pitch\
        +2*Bi_z0*Ci_z0*sin_q_pitch*cos_q_roll*cos_theta\
        -2*Bi_z0*Ci_z0*sin_theta*cos_q_pitch*cos_q_roll\
        +2*Bi_z0*d_ankle*sin_q_pitch*cos_q_roll*cos_theta\
        -2*Bi_z0*d_ankle*sin_q_pitch*cos_theta\
        -2*Bi_z0*d_ankle*sin_theta*cos_q_pitch*cos_q_roll\
        +2*Bi_z0*d_ankle*sin_theta*cos_q_pitch;

    } else if (ankle_config == "PTR" || ankle_config == "ptr") {
      d_term = 0.0;

    }

  } else if (motor_config == "X" || motor_config == "x") {
    if (ankle_config == "RTP" || ankle_config == "rtp") {
      d_term = -2*Ai_y0*Ci_x0*sin_theta*cos_q_pitch\
        -2*Ai_y0*Ci_y0*sin_q_pitch*sin_q_roll*sin_theta\
        -2*Ai_y0*Ci_z0*sin_q_pitch*sin_theta*cos_q_roll\
        -2*Ai_y0*d_ankle*sin_q_pitch*sin_theta*cos_q_roll\
        +2*Ai_y0*d_ankle*sin_q_pitch*sin_theta\
        -2*Ai_z0*Ci_x0*cos_q_pitch*cos_theta\
        +2*Ai_z0*Ci_x0*cos_q_pitch\
        -2*Ai_z0*Ci_y0*sin_q_pitch*sin_q_roll*cos_theta\
        +2*Ai_z0*Ci_y0*sin_q_pitch*sin_q_roll\
        -2*Ai_z0*Ci_z0*sin_q_pitch*cos_q_roll*cos_theta\
        +2*Ai_z0*Ci_z0*sin_q_pitch*cos_q_roll\
        -2*Ai_z0*d_ankle*sin_q_pitch*cos_q_roll*cos_theta\
        +2*Ai_z0*d_ankle*sin_q_pitch*cos_q_roll\
        +2*Ai_z0*d_ankle*sin_q_pitch*cos_theta\
        -2*Ai_z0*d_ankle*sin_q_pitch\
        +2*Bi_x0*Ci_x0*sin_q_pitch\
        -2*Bi_x0*Ci_y0*sin_q_roll*cos_q_pitch\
        -2*Bi_x0*Ci_z0*cos_q_pitch*cos_q_roll\
        -2*Bi_x0*d_ankle*cos_q_pitch*cos_q_roll\
        +2*Bi_x0*d_ankle*cos_q_pitch\
        +2*Bi_y0*Ci_x0*sin_theta*cos_q_pitch\
        +2*Bi_y0*Ci_y0*sin_q_pitch*sin_q_roll*sin_theta\
        +2*Bi_y0*Ci_z0*sin_q_pitch*sin_theta*cos_q_roll\
        +2*Bi_y0*d_ankle*sin_q_pitch*sin_theta*cos_q_roll\
        -2*Bi_y0*d_ankle*sin_q_pitch*sin_theta\
        +2*Bi_z0*Ci_x0*cos_q_pitch*cos_theta\
        +2*Bi_z0*Ci_y0*sin_q_pitch*sin_q_roll*cos_theta\
        +2*Bi_z0*Ci_z0*sin_q_pitch*cos_q_roll*cos_theta\
        +2*Bi_z0*d_ankle*sin_q_pitch*cos_q_roll*cos_theta\
        -2*Bi_z0*d_ankle*sin_q_pitch*cos_theta;

    } else if (ankle_config == "PTR" || ankle_config == "ptr") {
      d_term = 0.0;

    }
  }
  return d_term;
}

// 获取雅可比矩阵 ////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix2d getJacobian(const std::string& motor_config,
                            const std::string& ankle_config,
                            const Eigen::Vector3d& r_A1_0,
                            const Eigen::Vector3d& r_A2_0,
                            const Eigen::Vector3d& r_B1_0,
                            const Eigen::Vector3d& r_B2_0,
                            const Eigen::Vector3d& r_C1_0,
                            const Eigen::Vector3d& r_C2_0,
                            const double d_ankle,
                            const Eigen::Vector2d& ankle_pos,
                            const Eigen::Vector2d& motor_pos) {

  double d_ankle_2 = pow(d_ankle, 2);

  double sin_q_roll = sin(ankle_pos(0));
  double cos_q_roll = cos(ankle_pos(0));
  double sin_q_pitch = sin(ankle_pos(1));
  double cos_q_pitch = cos(ankle_pos(1));

  double a1 = getJacobianABTerm(
            motor_config, ankle_config,
            r_A1_0, r_B1_0, r_C1_0,
            d_ankle, d_ankle_2,
            sin_q_roll, cos_q_roll, sin_q_pitch, cos_q_pitch,
            motor_pos(0));
  double c1 = getJacobianCTerm(
            motor_config, ankle_config,
            r_A1_0, r_B1_0, r_C1_0,
            d_ankle, d_ankle_2,
            sin_q_roll, cos_q_roll, sin_q_pitch, cos_q_pitch,
            motor_pos(0));
  double d1 = getJacobianDTerm(
            motor_config, ankle_config,
            r_A1_0, r_B1_0, r_C1_0,
            d_ankle, d_ankle_2,
            sin_q_roll, cos_q_roll, sin_q_pitch, cos_q_pitch,
            motor_pos(0));

  double b2 = getJacobianABTerm(
            motor_config, ankle_config,
            r_A2_0, r_B2_0, r_C2_0,
            d_ankle, d_ankle_2,
            sin_q_roll, cos_q_roll, sin_q_pitch, cos_q_pitch,
            motor_pos(1));
  double c2 = getJacobianCTerm(
            motor_config, ankle_config,
            r_A2_0, r_B2_0, r_C2_0,
            d_ankle, d_ankle_2,
            sin_q_roll, cos_q_roll, sin_q_pitch, cos_q_pitch,
            motor_pos(1));
  double d2 = getJacobianDTerm(
            motor_config, ankle_config,
            r_A2_0, r_B2_0, r_C2_0,
            d_ankle, d_ankle_2,
            sin_q_roll, cos_q_roll, sin_q_pitch, cos_q_pitch,
            motor_pos(1));

  Eigen::Matrix2d jacobian = Eigen::Matrix2d::Identity();

  jacobian(0, 0) = -c1/a1;
  jacobian(0, 1) = -d1/a1;
  jacobian(1, 0) = -c2/b2;
  jacobian(1, 1) = -d2/b2;

  return jacobian;
}
