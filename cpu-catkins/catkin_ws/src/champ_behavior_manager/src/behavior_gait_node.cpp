#include "champ_behavior_manager/behavior_gait_node.h"

#include <sstream>

namespace champ_behavior_manager
{

BehaviorGaitNode::BehaviorGaitNode()
  : nh_()
  , pnh_("~")
{
  pnh_.param<std::string>("input_cmd_vel_topic", input_cmd_vel_topic_, "cmd_vel");
  pnh_.param<std::string>("output_cmd_vel_topic", output_cmd_vel_topic_, "cmd_vel/behavior");
  pnh_.param<std::string>("behavior_cmd_topic", behavior_cmd_topic_, "behavior_cmd");
  pnh_.param<std::string>("default_gait", default_gait_, "walk");

  behavior_sm_.setManualVelocityGait(default_gait_);

  // Load gait profiles from the private namespace "gaits".
  gait_manager_.loadFromRos(pnh_, "gaits");

  behavior_cmd_sub_ = nh_.subscribe<std_msgs::String>(
      behavior_cmd_topic_, 1, &BehaviorGaitNode::behaviorCmdCallback, this);

  raw_cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(
      input_cmd_vel_topic_, 1, &BehaviorGaitNode::rawCmdVelCallback, this);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(output_cmd_vel_topic_, 1);
  behavior_state_pub_ = nh_.advertise<std_msgs::String>("behavior_state", 1, true);
  gait_state_pub_ = nh_.advertise<std_msgs::String>("gait_state", 1, true);

  ROS_INFO_STREAM("BehaviorGaitNode: listening for raw velocity on '"
                  << input_cmd_vel_topic_ << "', publishing to '"
                  << output_cmd_vel_topic_ << "'");
}

void BehaviorGaitNode::spin()
{
  ros::Rate rate(50.0);
  while (ros::ok())
  {
    ros::spinOnce();
    publishCommand();
    rate.sleep();
  }
}

void BehaviorGaitNode::behaviorCmdCallback(const std_msgs::String::ConstPtr &msg)
{
  behavior_sm_.setBehaviorByName(msg->data);

  std_msgs::String state_msg;
  state_msg.data = behavior_sm_.stateName();
  behavior_state_pub_.publish(state_msg);

  std_msgs::String gait_msg;
  gait_msg.data = behavior_sm_.activeGait();
  gait_state_pub_.publish(gait_msg);
}

void BehaviorGaitNode::rawCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  last_raw_cmd_ = *msg;
  has_raw_cmd_ = true;
}

void BehaviorGaitNode::publishCommand()
{
  geometry_msgs::Twist output;

  if (!has_raw_cmd_ || behavior_sm_.state() == BehaviorStateMachine::State::Idle)
  {
    // Publish zero velocity in Idle or if we have never seen a command.
    cmd_vel_pub_.publish(output);
    return;
  }

  switch (behavior_sm_.state())
  {
  case BehaviorStateMachine::State::ManualVelocity:
  case BehaviorStateMachine::State::WalkToTarget:
  case BehaviorStateMachine::State::Patrol:
  default:
  {
    const std::string gait = behavior_sm_.activeGait().empty() ? default_gait_ : behavior_sm_.activeGait();
    output = gait_manager_.applyGait(last_raw_cmd_, gait);
    break;
  }
  }

  cmd_vel_pub_.publish(output);
}

}  // namespace champ_behavior_manager

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_gait_manager");
  champ_behavior_manager::BehaviorGaitNode node;
  node.spin();
  return 0;
}


