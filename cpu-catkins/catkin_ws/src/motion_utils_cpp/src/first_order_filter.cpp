#include "motion_utils_cpp/first_order_filter.h"

#include <algorithm>

namespace motion_utils_cpp
{

FirstOrderFilter::FirstOrderFilter(double time_constant, double dt, double initial)
  : tau_(time_constant)
  , dt_(dt)
  , alpha_(0.0)
  , y_(initial)
{
  computeAlpha();
}

void FirstOrderFilter::setState(double value)
{
  y_ = value;
}

double FirstOrderFilter::state() const
{
  return y_;
}

double FirstOrderFilter::update(double measurement)
{
  y_ = y_ + alpha_ * (measurement - y_);
  return y_;
}

void FirstOrderFilter::computeAlpha()
{
  if (dt_ <= 0.0)
  {
    alpha_ = 0.0;
    return;
  }

  if (tau_ <= 0.0)
  {
    alpha_ = 1.0;
    return;
  }

  alpha_ = dt_ / (tau_ + dt_);
  alpha_ = std::max(0.0, std::min(1.0, alpha_));
}

}  // namespace motion_utils_cpp


