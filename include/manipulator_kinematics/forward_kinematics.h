#ifndef MANIPULATOR_KINEMATICS_FORWARD_KINEMATICS_H
#define MANIPULATOR_KINEMATICS_FORWARD_KINEMATICS_H

#include <vector>
#include <Eigen/Dense>
#include "manipulator_kinematics/robot_config.h"
#include "manipulator_kinematics/joint_state.h"
#include "manipulator_kinematics/ee_state.h"

/**
 * @brief Class to perform forward kinematics of a robotic manipulator.
 */
class ForwardKinematics
{
public:
  /**
   * @brief Constructor that initializes the ForwardKinematics with RobotConfig and JointState.
   * @param robot_config Robot configuration.
   */
  explicit ForwardKinematics(const RobotConfig& robot_config) : robot_config_(robot_config)
  {
  }

  /**
   * @brief Computes the end-effector state of the robot arm.
   * @param joint_state Joint state.
   * @return End-effector state.
   */
  [[nodiscard]] EEState computeEEState(const JointState& joint_state) const;

  // Get the robot configuration
  [[nodiscard]] const RobotConfig& getRobotConfig() const
  {
    return robot_config_;
  }

private:
  RobotConfig robot_config_;  ///< Robot configuration
  //   const RobotConfig& robot_config_;  ///< Robot configuration //FIXME: should this be a reference?
};

#endif  // MANIPULATOR_KINEMATICS_FORWARD_KINEMATICS_H