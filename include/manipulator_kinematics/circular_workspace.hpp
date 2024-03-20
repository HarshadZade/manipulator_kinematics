#ifndef MANIPULATOR_KINEMATICS_CIRCULAR_WORKSPACE_H
#define MANIPULATOR_KINEMATICS_CIRCULAR_WORKSPACE_H

#include <cmath>

#include "manipulator_kinematics/ee_state.hpp"
#include "manipulator_kinematics/workspace.hpp"

/**
 * @class CircularWorkspace
 * @brief Represents a circular workspace for a robotic manipulator.
 * This class defines a circular area representing the workspace within which
 * the end-effector of a robotic manipulator can operate. It extends the
 * Workspace class, providing a specific implementation for checking if a given
 * end-effector state is within this circular workspace.
 */
class CircularWorkspace : public Workspace {
 public:
  /**
   * @brief Constructs a CircularWorkspace with a specified radius and center.
   * @param radius The radius of the circular workspace.
   * @param center_x The x-coordinate of the center of the circular workspace.
   * @param center_y The y-coordinate of the center of the circular workspace.
   */
  explicit CircularWorkspace(double radius, double center_x, double center_y)
      : radius_(radius), center_x_(center_x), center_y_(center_y) {}

  /**
   * @brief Checks if the given end-effector state is within the workspace.
   *
   * This method overrides the isWithinWorkspace method from the Workspace
   * base class. It calculates if the given end-effector state is within
   * the bounds of this circular workspace.
   * @param ee_state The state of the end-effector to check.
   * @return true if the end-effector is within the workspace, false otherwise.
   */
  [[nodiscard]] bool isWithinWorkspace(const EEState& ee_state) const override;

 private:
  double radius_;    ///< Radius of the circular workspace
  double center_x_;  ///< x-coordinate of the center of the circular workspace
  double center_y_;  ///< y-coordinate of the center of the circular workspace
};

#endif  // MANIPULATOR_KINEMATICS_CIRCULAR_WORKSPACE_H
