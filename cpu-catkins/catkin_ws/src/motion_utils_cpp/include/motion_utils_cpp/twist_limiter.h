#pragma once

namespace motion_utils_cpp
{

/**
 * @brief Simple 2D twist representation, independent of ROS messages.
 */
struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

/**
 * @brief Rate limits and clamps 2D twist commands.
 *
 * This is a small helper that can be used from pure C++ code or
 * wrapped by a ROS node that converts to/from geometry_msgs/Twist.
 */
class TwistLimiter
{
public:
  TwistLimiter();

  void setMaxVelocity(double max_vx, double max_vy, double max_wz);

  void setMaxAcceleration(double max_ax, double max_ay, double max_awz);

  /// Apply rate limiting to the desired command given the previous command and dt.
  Twist2D limit(const Twist2D &desired, const Twist2D &previous, double dt) const;

private:
  double max_vx_;
  double max_vy_;
  double max_wz_;

  double max_ax_;
  double max_ay_;
  double max_awz_;
};

}  // namespace motion_utils_cpp


