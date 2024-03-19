#include "manipulator_kinematics/workspace.hpp"

#include <cmath>
#include <iostream>

#include "manipulator_kinematics/ee_state.hpp"

// Checks if the end-effector is within the workspace
bool Workspace::isWithinWorkspace(const EEState& ee_state) const {
  // Check if the end-effector is within the circular workspace
  double distance = std::hypot(ee_state.x - circular_workspace_.center_x,
                               ee_state.y - circular_workspace_.center_y);
  return distance <= circular_workspace_.radius;
}
