#include "manipulator_kinematics/circular_workspace.hpp"

#include <cmath>
#include <iostream>

#include "manipulator_kinematics/ee_state.hpp"

bool CircularWorkspace::isWithinWorkspace(const EEState& ee_state) const {
  // Check if the end-effector is within the circular workspace
  double distance =
      std::hypot(ee_state.x - this->center_x_, ee_state.y - this->center_y_);
  return distance <= this->radius_;
}
