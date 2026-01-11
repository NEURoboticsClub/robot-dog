#pragma once

namespace motion_utils_cpp
{

/**
 * @brief Basic PID controller with output and integrator limits.
 *
 * This implementation is intentionally self-contained and easy to test.
 */
class PidController
{
public:
  PidController(double kp = 0.0, double ki = 0.0, double kd = 0.0);

  void setGains(double kp, double ki, double kd);

  void setOutputLimits(double min_output, double max_output);

  void setIntegratorLimits(double min_integral, double max_integral);

  void reset();

  /// Compute control output given setpoint, measurement, and timestep.
  double update(double setpoint, double measurement, double dt);

  double proportionalTerm() const { return p_term_; }
  double integralTerm() const { return i_term_; }
  double derivativeTerm() const { return d_term_; }

private:
  double kp_;
  double ki_;
  double kd_;

  double min_output_;
  double max_output_;

  double min_integral_;
  double max_integral_;

  double integrator_;
  double prev_error_;
  bool has_prev_error_;

  double p_term_;
  double i_term_;
  double d_term_;
};

}  // namespace motion_utils_cpp


