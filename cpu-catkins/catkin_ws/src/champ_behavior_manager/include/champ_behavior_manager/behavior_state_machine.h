#pragma once

#include <string>

namespace champ_behavior_manager
{
/**
 * @brief Simple behavior state machine for the quadruped.
 *
 * For this demo we keep the logic intentionally simple but structured:
 * - Idle: robot should not move.
 * - ManualVelocity: forward raw teleop velocity (with gait scaling).
 * - WalkToTarget / Patrol: reserved for more advanced behaviors.
 */
class BehaviorStateMachine
{
public:
  enum class State
  {
    Idle = 0,
    ManualVelocity,
    WalkToTarget,
    Patrol
  };

  BehaviorStateMachine();

  /// Set the desired behavior by name (e.g. "idle", "manual", "walk_to", "patrol").
  void setBehaviorByName(const std::string &behavior_name);

  /// Return the current state.
  State state() const { return state_; }

  /// Human-readable name of the current state.
  std::string stateName() const;

  /// Name of the active gait for this behavior.
  const std::string &activeGait() const { return active_gait_; }

  /// Override the gait associated with ManualVelocity.
  void setManualVelocityGait(const std::string &gait_name);

private:
  State state_;
  std::string active_gait_;
  std::string manual_velocity_gait_;
};

}  // namespace champ_behavior_manager


