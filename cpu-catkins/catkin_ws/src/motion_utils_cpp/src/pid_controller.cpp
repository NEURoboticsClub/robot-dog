#include "motion_utils_cpp/pid_controller.h"

#include <algorithm>

namespace motion_utils_cpp
{

PidController::PidController(double kp, double ki, double kd)
  : kp_(kp)
  , ki_(ki)
  , kd_(kd)
  , min_output_(-1e9)
  , max_output_(1e9)
  , min_integral_(-1e9)
  , max_integral_(1e9)
  , integrator_(0.0)
  , prev_error_(0.0)
  , has_prev_error_(false)
  , p_term_(0.0)
  , i_term_(0.0)
  , d_term_(0.0)
{
}

void PidController::setGains(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PidController::setOutputLimits(double min_output, double max_output)
{
  min_output_ = min_output;
  max_output_ = max_output;
}

void PidController::setIntegratorLimits(double min_integral, double max_integral)
{
  min_integral_ = min_integral;
  max_integral_ = max_integral;
}

void PidController::reset()
{
  integrator_ = 0.0;
  prev_error_ = 0.0;
  has_prev_error_ = false;
  p_term_ = 0.0;
  i_term_ = 0.0;
  d_term_ = 0.0;
}

double PidController::update(double setpoint, double measurement, double dt)
{
  if (dt <= 0.0)
  {
    return 0.0;
  }

  const double error = setpoint - measurement;

  p_term_ = kp_ * error;

  integrator_ += error * dt;
  integrator_ = std::max(min_integral_, std::min(max_integral_, integrator_));
  i_term_ = ki_ * integrator_;

  if (has_prev_error_)
  {
    const double derivative = (error - prev_error_) / dt;
    d_term_ = kd_ * derivative;
  }
  else
  {
    d_term_ = 0.0;
    has_prev_error_ = true;
  }

  prev_error_ = error;

  double output = p_term_ + i_term_ + d_term_;
  output = std::max(min_output_, std::min(max_output_, output));
  return output;
}

}  // namespace motion_utils_cpp


