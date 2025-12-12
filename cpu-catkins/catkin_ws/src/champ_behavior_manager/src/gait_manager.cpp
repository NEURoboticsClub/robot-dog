#include "champ_behavior_manager/gait_manager.h"

#include <ros/node_handle.h>

namespace champ_behavior_manager
{

bool GaitManager::loadFromRos(const ros::NodeHandle &nh, const std::string &param_ns)
{
  ros::NodeHandle gaits_nh(nh, param_ns);

  std::vector<std::string> gait_names;
  if (!gaits_nh.getParamNames(gait_names))
  {
    ROS_WARN_STREAM("GaitManager: no parameters found under namespace '" << gaits_nh.getNamespace() << "'");
    return false;
  }

  gaits_.clear();

  for (const auto &full_name : gait_names)
  {
    // Expect parameters of the form <ns>/<gait_name>/param.
    // Extract gait_name by removing the namespace prefix.
    std::string ns = gaits_nh.getNamespace();
    std::string gait_name = full_name;
    if (gait_name.find(ns + "/") == 0)
    {
      gait_name = gait_name.substr(ns.size() + 1);
    }

    // Only keep leaf names (ignore nested parameters).
    if (gait_name.find('/') != std::string::npos)
    {
      continue;
    }

    ros::NodeHandle gait_nh(gaits_nh, gait_name);

    GaitProfile profile(gait_name);
    gait_nh.param("linear_x_scale", profile.linear_x_scale, 1.0);
    gait_nh.param("linear_y_scale", profile.linear_y_scale, 1.0);
    gait_nh.param("angular_z_scale", profile.angular_z_scale, 1.0);
    gait_nh.param("body_height_offset", profile.body_height_offset, 0.0);
    gait_nh.param("swing_height", profile.swing_height, 0.0);
    gait_nh.param("stance_duration", profile.stance_duration, 0.0);

    gaits_[profile.name] = profile;
  }

  if (gaits_.empty())
  {
    ROS_WARN_STREAM("GaitManager: loaded no gaits from namespace '" << gaits_nh.getNamespace() << "'");
    return false;
  }

  ROS_INFO_STREAM("GaitManager: loaded " << gaits_.size() << " gait profiles");
  return true;
}

void GaitManager::setGait(const GaitProfile &profile)
{
  gaits_[profile.name] = profile;
}

bool GaitManager::hasGait(const std::string &name) const
{
  return gaits_.find(name) != gaits_.end();
}

bool GaitManager::getGait(const std::string &name, GaitProfile &out) const
{
  auto it = gaits_.find(name);
  if (it == gaits_.end())
  {
    return false;
  }
  out = it->second;
  return true;
}

geometry_msgs::Twist GaitManager::applyGait(const geometry_msgs::Twist &input,
                                            const std::string &gait_name) const
{
  geometry_msgs::Twist result = input;

  GaitProfile profile;
  if (!getGait(gait_name, profile))
  {
    // Unknown gait: return input unchanged.
    return result;
  }

  result.linear.x *= profile.linear_x_scale;
  result.linear.y *= profile.linear_y_scale;
  result.angular.z *= profile.angular_z_scale;

  return result;
}

}  // namespace champ_behavior_manager


