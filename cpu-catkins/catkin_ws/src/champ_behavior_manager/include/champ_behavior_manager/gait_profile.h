#pragma once

#include <string>

namespace champ_behavior_manager
{
/**
 * @brief Represents a single gait profile with simple scaling parameters.
 *
 * This is intentionally lightweight. It is not tied to ROS so it can
 * be unit-tested in isolation.
 */
struct GaitProfile
{
  std::string name;

  // Scale factors for velocity components.
  double linear_x_scale{1.0};
  double linear_y_scale{1.0};
  double angular_z_scale{1.0};

  // Simple body posture parameters (for documentation / future use).
  double body_height_offset{0.0};
  double swing_height{0.0};
  double stance_duration{0.0};

  GaitProfile() = default;

  explicit GaitProfile(const std::string &name_) : name(name_) {}
};

}  // namespace champ_behavior_manager


