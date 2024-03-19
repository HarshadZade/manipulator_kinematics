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

  // Function to convert an angular velocity vector into a skew-symmetric matrix
  [[nodiscard]] const Eigen::Matrix3d omegaHat(const Eigen::Vector3d& omega) const;

  // Function to compute the Rodrigues' rotation formula
  [[nodiscard]] const Eigen::Matrix3d rodriguesRotation(const Eigen::Matrix3d& hatOmega, double theta) const;

  // Function to calculate the translation vector for a screw motion
  [[nodiscard]] const Eigen::Vector3d calculateTranslation(const Eigen::Matrix3d& R, const Eigen::Matrix3d& hatOmega,
                                                           const Eigen::Vector3d& v, double theta) const;

  // Main function to compute the exponential of a twist
  [[nodiscard]] const Eigen::Matrix4d expTwist(const Eigen::VectorXd& screwAxis, double theta) const;

  // Forward kinematics function
  [[nodiscard]] const Eigen::Matrix4d computeFK(const Eigen::MatrixXd& screwAxes, const Eigen::VectorXd& thetas) const;

  // Get screw axis for a revolute joint
  [[nodiscard]] const Eigen::VectorXd getRevoluteScrewAxis(const Eigen::Vector3d& w, const Eigen::Vector3d& q) const;

  // Function for getting screwAxes for the robot
  [[nodiscard]] const Eigen::MatrixXd getScrewAxes() const;
};

#endif  // MANIPULATOR_KINEMATICS_FORWARD_KINEMATICS_H