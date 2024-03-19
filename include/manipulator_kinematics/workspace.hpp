#ifndef MANIPULATOR_KINEMATICS_WORKSPACE_H
#define MANIPULATOR_KINEMATICS_WORKSPACE_H

#include "manipulator_kinematics/ee_state.h"

/**
 * @brief Structure to represent a circular workspace.
 */
struct CircularWorkspace
{
  double radius;    ///< Radius of the circular workspace
  double center_x;  ///< x-coordinate of the center of the circular workspace
  double center_y;  ///< y-coordinate of the center of the circular workspace
};

/**
 * @brief Class to represent the workspace of a robotic manipulator.
 */
class Workspace
{
public:
  /**
   * @brief Constructor that initializes the Workspace with a circular
   * workspace.
   * @param circular_workspace Circular workspace.
   */
  explicit Workspace(const CircularWorkspace& circular_workspace) : circular_workspace_(circular_workspace)
  {
  }

  /**
   * @brief Checks if the end-effector is within the workspace.
   * @param EEState End-effector state.
   * @return True if the end-effector is within the workspace, false otherwise.
   */
  [[nodiscard]] bool isWithinWorkspace(const EEState& ee_state) const;

  // Set the circular workspace
  void setCircularWorkspace(const CircularWorkspace& circular_workspace)
  {
    circular_workspace_ = circular_workspace;
  }

private:
  CircularWorkspace circular_workspace_;  ///< Circular workspace
};

#endif  // MANIPULATOR_KINEMATICS_WORKSPACE_H