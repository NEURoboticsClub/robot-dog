#pragma once

#include <map>
#include <string>

#include <geometry_msgs/Twist.h>

#include "champ_behavior_manager/gait_profile.h"

namespace champ_behavior_manager
{
/**
 * @brief Manages a collection of gait profiles and applies them to Twist commands.
 */
class GaitManager
{
public:
  using GaitMap = std::map<std::string, GaitProfile>;

  /// Load gaits from a ROS parameter tree under the provided namespace.
  /// Expected layout:
  /// gaits:
  ///   walk:
  ///     linear_x_scale: 0.5
  ///     ...
  bool loadFromRos(const ros::NodeHandle &nh, const std::string &param_ns);

  /// Add or overwrite a gait profile.
  void setGait(const GaitProfile &profile);

  /// Returns true if a gait with the given name exists.
  bool hasGait(const std::string &name) const;

  /// Get a gait profile by name. Returns false if not found.
  bool getGait(const std::string &name, GaitProfile &out) const;

  /// Apply the given gait to an input twist, returning a new scaled twist.
  geometry_msgs::Twist applyGait(const geometry_msgs::Twist &input,
                                 const std::string &gait_name) const;

  const GaitMap &gaits() const { return gaits_; }

private:
  GaitMap gaits_;
};

}  // namespace champ_behavior_manager


