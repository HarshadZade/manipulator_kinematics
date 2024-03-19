#include <cmath>
#include "manipulator_kinematics/inverse_kinematics.h"

// Computes the joint states given the desired end-effector state
JointState InverseKinematics::computeJointState(const EEState& ee_state) const
{
  // Compute the joint states
  // TODO: Implement this
  JointState joint_state(robot_config_);
  joint_state.setJointAngles({ 0, 0, 0 });
  return joint_state;
}