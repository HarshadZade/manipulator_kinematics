#ifndef MANIPULATOR_KINEMATICS_EE_STATE_H
#define MANIPULATOR_KINEMATICS_EE_STATE_H

/**
 * @struct EEState
 * @brief Represents the state of the end-effector in a robotic manipulator system.
 * This structure is used to store the position and orientation of the end-effector
 * relative to some reference frame. It includes the x and y coordinates for position,
 * and thetaP for the orientation angle.
 */
struct EEState
{
  double x{ 0.0 };       ///< X-coordinate of the end-effector's position.
  double y{ 0.0 };       ///< Y-coordinate of the end-effector's position.
  double thetaP{ 0.0 };  ///< Orientation angle of the end-effector.
};

#endif  // MANIPULATOR_KINEMATICS_EE_STATE_H