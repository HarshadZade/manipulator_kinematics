#include "manipulator_kinematics/joint_state.hpp"

#include <sstream>
#include <stdexcept>

// Set the joint angles, ensuring they are within the joint limits
void JointState::setJointAngles(const std::vector<double>& angles) {
  // Check if the angles are within the joint limits
  for (int i = 0; i < angles.size(); ++i) {
    const Joint& joint = robot_config_.getJoints().at(i);
    if (angles.at(i) < joint.theta_min || angles.at(i) > joint.theta_max) {
      std::ostringstream msg;
      msg << "Joint angle is out of range: Joint ID = " << i
          << ", Angle = " << angles.at(i);
      throw std::out_of_range(msg.str());
    }
  }
  joint_angles_ = angles;
}

// Retrieve the joint angles
const std::vector<double>& JointState::getJointAngles() const noexcept {
  return joint_angles_;
}
