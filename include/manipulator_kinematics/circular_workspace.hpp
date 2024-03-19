#ifndef MANIPULATOR_KINEMATICS_CIRCULAR_WORKSPACE_H
#define MANIPULATOR_KINEMATICS_CIRCULAR_WORKSPACE_H

#include "manipulator_kinematics/ee_state.hpp"
#include "manipulator_kinematics/workspace.hpp"
#include <cmath>


class CircularWorkspace : public Workspace
{
public:
  explicit CircularWorkspace(double radius, double center_x, double center_y)
    : radius_(radius), center_x_(center_x), center_y_(center_y)
  {
  }
  [[nodiscard]] bool isWithinWorkspace(const EEState& ee_state) const override;

private:
  double radius_;    ///< Radius of the circular workspace
  double center_x_;  ///< x-coordinate of the center of the circular workspace
  double center_y_;  ///< y-coordinate of the center of the circular workspace
};

#endif // MANIPULATOR_KINEMATICS_CIRCULAR_WORKSPACE_H
