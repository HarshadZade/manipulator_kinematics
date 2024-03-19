#ifndef MANIPULATOR_KINEMATICS_INVERSE_KINEMATICS_H
#define MANIPULATOR_KINEMATICS_INVERSE_KINEMATICS_H

#include "manipulator_kinematics/ee_state.h"
#include "manipulator_kinematics/joint_state.h"
#include "manipulator_kinematics/robot_config.h"

/**
 * @class InverseKinematics
 * @brief Performs inverse kinematics calculations for a robotic manipulator.
 * This class provides the functionality to compute the joint states of a robot
 * based on its end-effector state and robot configuration.
 */
class InverseKinematics
{
public:
  /**
   * @brief Constructs an InverseKinematics object with the given robot
   * configuration.
   * @param robot_config The configuration of the robot, including link lengths
   * and joint limits.
   */
  explicit InverseKinematics(const RobotConfig& robot_config) : robot_config_(robot_config)
  {
  }

  /**
   * @brief Computes the joint states given the desired end-effector state.
   * @param ee_state The desired end-effector state, including position and
   * orientation.
   * @return A vector of all possible joint states that achieve the desired
   * end-effector state.
   */
  [[nodiscard]] std::vector<JointState> computeJointState(const EEState& ee_state) const;

  /**
   * @brief Retrieves the robot configuration used by this instance.
   * @return A constant reference to the robot's configuration.
   */
  [[nodiscard]] const RobotConfig& getRobotConfig() const
  {
    return robot_config_;
  }

private:
  RobotConfig robot_config_;  ///< Robot configuration
};

#endif  // MANIPULATOR_KINEMATICS_INVERSE_KINEMATICS_H