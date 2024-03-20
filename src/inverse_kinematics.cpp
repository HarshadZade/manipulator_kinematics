#include "manipulator_kinematics/inverse_kinematics.hpp"  // cppcheck-suppress legal/copyright

#include <cmath>

// Computes the joint states given the desired end-effector state
std::vector<JointState> InverseKinematics::computeJointState(
    const EEState& ee_state) const {
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
  if (distance > l1 + l2 + l3) {
    throw std::out_of_range("The given pose is not reachable");
  }

  std::vector<JointState> joint_states;
  JointState joint_state(robot_config_);

  // Compute inverse kinematics
  double x_ = x - l3 * cos(thetaP);
  double y_ = y - l3 * sin(thetaP);
  double cos_theta2 = (x_ * x_ + y_ * y_ - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  double sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2);
  for (int i = 0; i < 2; i++) {
    double theta2 = atan2((i == 0 ? 1 : -1) * sin_theta2, cos_theta2);
    double theta1 =
        atan2(y_, x_) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
    double theta3 = thetaP - (theta1 + theta2);

    joint_state.setJointAngles({theta1, theta2, theta3});
    joint_states.push_back(joint_state);
  }

  return joint_states;
}
