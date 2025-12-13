#pragma once

namespace motion_utils_cpp
{

/**
 * @brief Simple first-order (exponential) low-pass filter.
 *
 * The discrete-time form is:
 *   y_k = y_{k-1} + alpha * (x_k - y_{k-1})
 *
 * where alpha is computed from the continuous-time time constant tau and
 * sample period dt:
 *   alpha = dt / (tau + dt)
 */
class FirstOrderFilter
{
public:
  FirstOrderFilter(double time_constant, double dt, double initial = 0.0);

  /// Set the current state of the filter directly.
  void setState(double value);

  /// Return the current state.
  double state() const;

  /// Update the filter with a new measurement and return the new state.
  double update(double measurement);

  /// Return the configured time constant.
  double timeConstant() const { return tau_; }

  /// Return the configured sample period.
  double samplePeriod() const { return dt_; }

private:
  double tau_;
  double dt_;
  double alpha_;
  double y_;

  void computeAlpha();
};

}  // namespace motion_utils_cpp


