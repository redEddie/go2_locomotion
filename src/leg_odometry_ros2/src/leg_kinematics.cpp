#include "leg_odometry_ros2/leg_kinematics.hpp"

#include <cmath>

namespace leg_odometry_ros2
{
namespace
{
constexpr double kHipToThighOffset = 0.08505;
constexpr double kThighLength = 0.213;
constexpr double kCalfLength = 0.213;
}

Eigen::Vector3d LegKinematics::hip_offset(LegId leg) const
{
  switch (leg)
  {
    case LegId::FL:
      return Eigen::Vector3d(0.1934, 0.0465, 0.0);
    case LegId::FR:
      return Eigen::Vector3d(0.1934, -0.0465, 0.0);
    case LegId::RL:
      return Eigen::Vector3d(-0.1934, 0.0465, 0.0);
    case LegId::RR:
      return Eigen::Vector3d(-0.1934, -0.0465, 0.0);
    default:
      return Eigen::Vector3d::Zero();
  }
}

double LegKinematics::hip_sign(LegId leg) const
{
  return (leg == LegId::FL || leg == LegId::RL) ? 1.0 : -1.0;
}

Eigen::Vector3d LegKinematics::compute_foot_position(const Eigen::Vector3d &q, LegId leg) const
{
  const double hip = q[0];
  const double thigh = q[1];
  const double calf = q[2];

  const double c_hip = std::cos(hip);
  const double s_hip = std::sin(hip);
  const double c_thigh = std::cos(thigh);
  const double s_thigh = std::sin(thigh);
  const double c_thigh_calf = std::cos(thigh + calf);
  const double s_thigh_calf = std::sin(thigh + calf);

  Eigen::Matrix3d rot_x;
  rot_x << 1.0, 0.0, 0.0,
           0.0, c_hip, -s_hip,
           0.0, s_hip, c_hip;

  Eigen::Vector3d hip_to_thigh(0.0, hip_sign(leg) * kHipToThighOffset, 0.0);
  Eigen::Vector3d thigh_vec = Eigen::Vector3d(s_thigh * -kThighLength, 0.0, c_thigh * -kThighLength);
  Eigen::Vector3d calf_vec = Eigen::Vector3d(s_thigh_calf * -kCalfLength, 0.0, c_thigh_calf * -kCalfLength);

  return hip_offset(leg) + rot_x * (hip_to_thigh + thigh_vec + calf_vec);
}

Eigen::Matrix3d LegKinematics::compute_foot_jacobian(const Eigen::Vector3d &q, LegId leg, double eps) const
{
  Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; ++i)
  {
    Eigen::Vector3d q_plus = q;
    Eigen::Vector3d q_minus = q;
    q_plus[i] += eps;
    q_minus[i] -= eps;
    const Eigen::Vector3d p_plus = compute_foot_position(q_plus, leg);
    const Eigen::Vector3d p_minus = compute_foot_position(q_minus, leg);
    J.col(i) = (p_plus - p_minus) / (2.0 * eps);
  }
  return J;
}

}  // namespace leg_odometry_ros2
