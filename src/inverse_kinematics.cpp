#include <cmath>
#include "manipulator_kinematics/inverse_kinematics.h"

// Computes the joint states given the desired end-effector state
JointState InverseKinematics::computeJointState(const EEState& ee_state) const  // TODO: Handle multiple solutions
{
  // Get end effector coordinates and orientation
  double x = ee_state.x;
  double y = ee_state.y;
  double thetaP = ee_state.thetaP;

  // Get link lengths
  double l1 = robot_config_.getLinks()[0].length;
  double l2 = robot_config_.getLinks()[1].length;
  double l3 = robot_config_.getLinks()[2].length;

  // Check if the given pose is reachable
  double distance = std::hypot(x, y);
  if (distance > l1 + l2 + l3)
  {
    throw std::out_of_range("The given pose is not reachable");
  }

  // Compute inverse kinematics
  double x_ = x - l3 * cos(thetaP);
  double y_ = y - l3 * sin(thetaP);
  double theta2 = acos((x_ * x_ + y_ * y_ - (l1 * l1 + l2 * l2)) / (2 * l1 * l2));
  double theta1 = atan2(y_, x_) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
  double theta3 = thetaP - (theta1 + theta2);

  // Create a joint state object
  JointState joint_state(robot_config_);
  joint_state.setJointAngles({ theta1, theta2, theta3 });

  return joint_state;
}