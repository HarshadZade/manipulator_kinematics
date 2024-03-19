#ifndef MANIPULATOR_KINEMATICS_EE_STATE_H
#define MANIPULATOR_KINEMATICS_EE_STATE_H

/**
 * @brief Structure to represent the state of the end-effector.
 */
struct EEState
{
  double x{ 0.0 }, y{ 0.0 }, thetaP{ 0.0 };  ///< End-effector state
};

#endif  // MANIPULATOR_KINEMATICS_EE_STATE_H