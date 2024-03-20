#ifndef MANIPULATOR_KINEMATICS_WORKSPACE_H
#define MANIPULATOR_KINEMATICS_WORKSPACE_H

#include "manipulator_kinematics/ee_state.hpp"

/**
 * @class Workspace
 * @brief Abstract base class to represent the workspace of a robotic
 * manipulator. Defines a general interface for workspace classes, which
 * determine whether a given end-effector state is within the robot's
 * operational workspace. This class is intended to be subclassed with specific
 * implementations for different workspace geometries.
 */
class Workspace {
 public:
  /**
   * @brief Default constructor for the Workspace class.
   */
  Workspace() = default;

  /**
   * @brief Checks if the given end-effector state is within the workspace.
   * This is a pure virtual method that must be implemented by derived classes
   * to check if the specified end-effector state (position and orientation)
   * falls within the defined workspace of the robotic manipulator.
   * @param ee_state The state of the end-effector to be checked.
   * @return true if the end-effector state is within the workspace, false
   * otherwise.
   */
  [[nodiscard]] virtual bool isWithinWorkspace(
      const EEState& ee_state) const = 0;
};

#endif  // MANIPULATOR_KINEMATICS_WORKSPACE_H