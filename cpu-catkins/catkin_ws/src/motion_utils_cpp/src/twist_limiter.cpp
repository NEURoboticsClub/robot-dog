#include "motion_utils_cpp/twist_limiter.h"

#include <algorithm>
#include <cmath>

namespace motion_utils_cpp
{

TwistLimiter::TwistLimiter()
  : max_vx_(1.0)
  , max_vy_(1.0)
  , max_wz_(1.0)
  , max_ax_(1.0)
  , max_ay_(1.0)
  , max_awz_(1.0)
{
}

void TwistLimiter::setMaxVelocity(double max_vx, double max_vy, double max_wz)
{
  max_vx_ = std::fabs(max_vx);
  max_vy_ = std::fabs(max_vy);
  max_wz_ = std::fabs(max_wz);
}

void TwistLimiter::setMaxAcceleration(double max_ax, double max_ay, double max_awz)
{
  max_ax_ = std::fabs(max_ax);
  max_ay_ = std::fabs(max_ay);
  max_awz_ = std::fabs(max_awz);
}

Twist2D TwistLimiter::limit(const Twist2D &desired, const Twist2D &previous, double dt) const
{
  Twist2D result = previous;

  if (dt <= 0.0)
  {
    return result;
  }

  const auto clampDelta = [dt](double desired_val, double prev_val, double max_accel) {
    const double max_delta = max_accel * dt;
    const double delta = desired_val - prev_val;
    if (delta > max_delta)
    {
      return prev_val + max_delta;
    }
    if (delta < -max_delta)
    {
      return prev_val - max_delta;
    }
    return desired_val;
  };

  result.vx = clampDelta(desired.vx, previous.vx, max_ax_);
  result.vy = clampDelta(desired.vy, previous.vy, max_ay_);
  result.wz = clampDelta(desired.wz, previous.wz, max_awz_);

  result.vx = std::max(-max_vx_, std::min(max_vx_, result.vx));
  result.vy = std::max(-max_vy_, std::min(max_vy_, result.vy));
  result.wz = std::max(-max_wz_, std::min(max_wz_, result.wz));

  return result;
}

}  // namespace motion_utils_cpp


