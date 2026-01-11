#include "champ_behavior_manager/behavior_state_machine.h"

namespace champ_behavior_manager
{

BehaviorStateMachine::BehaviorStateMachine()
  : state_(State::Idle)
  , active_gait_("walk")
  , manual_velocity_gait_("walk")
{
}

void BehaviorStateMachine::setBehaviorByName(const std::string &behavior_name)
{
  std::string lower = behavior_name;
  for (char &c : lower)
  {
    c = static_cast<char>(::tolower(c));
  }

  if (lower == "idle")
  {
    state_ = State::Idle;
    active_gait_.clear();
  }
  else if (lower == "manual" || lower == "manual_velocity")
  {
    state_ = State::ManualVelocity;
    active_gait_ = manual_velocity_gait_;
  }
  else if (lower == "walk_to" || lower == "walktotarget")
  {
    state_ = State::WalkToTarget;
    active_gait_ = manual_velocity_gait_;
  }
  else if (lower == "patrol")
  {
    state_ = State::Patrol;
    active_gait_ = manual_velocity_gait_;
  }
}

std::string BehaviorStateMachine::stateName() const
{
  switch (state_)
  {
  case State::Idle:
    return "Idle";
  case State::ManualVelocity:
    return "ManualVelocity";
  case State::WalkToTarget:
    return "WalkToTarget";
  case State::Patrol:
    return "Patrol";
  default:
    return "Unknown";
  }
}

void BehaviorStateMachine::setManualVelocityGait(const std::string &gait_name)
{
  manual_velocity_gait_ = gait_name;
  if (state_ == State::ManualVelocity)
  {
    active_gait_ = manual_velocity_gait_;
  }
}

}  // namespace champ_behavior_manager


