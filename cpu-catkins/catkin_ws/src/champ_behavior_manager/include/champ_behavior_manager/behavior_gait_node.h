#pragma once

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include "champ_behavior_manager/behavior_state_machine.h"
#include "champ_behavior_manager/gait_manager.h"

namespace champ_behavior_manager
{
/**
 * @brief ROS-facing node that connects teleop / behavior commands to champ_base.
 */
class BehaviorGaitNode
{
public:
  BehaviorGaitNode();

  void spin();

private:
  void behaviorCmdCallback(const std_msgs::String::ConstPtr &msg);
  void rawCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  void publishCommand();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber behavior_cmd_sub_;
  ros::Subscriber raw_cmd_vel_sub_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher behavior_state_pub_;
  ros::Publisher gait_state_pub_;

  BehaviorStateMachine behavior_sm_;
  GaitManager gait_manager_;

  geometry_msgs::Twist last_raw_cmd_;
  bool has_raw_cmd_{false};

  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string behavior_cmd_topic_;
  std::string default_gait_;
};

}  // namespace champ_behavior_manager


