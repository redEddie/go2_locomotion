#pragma once

#include <Eigen/Core>

namespace leg_odometry_ros2
{

enum class LegId
{
  FR = 0,
  FL = 1,
  RR = 2,
  RL = 3
};

class LegKinematics
{
public:
  LegKinematics() = default;

  Eigen::Vector3d compute_foot_position(const Eigen::Vector3d &q, LegId leg) const;
  Eigen::Matrix3d compute_foot_jacobian(const Eigen::Vector3d &q, LegId leg, double eps) const;

private:
  Eigen::Vector3d hip_offset(LegId leg) const;
  double hip_sign(LegId leg) const;
};

}  // namespace leg_odometry_ros2
