#ifndef MANIPULATOR_KINEMATICS_FORWARD_KINEMATICS_H
#define MANIPULATOR_KINEMATICS_FORWARD_KINEMATICS_H

#include <Eigen/Dense>
#include <vector>

#include "manipulator_kinematics/ee_state.hpp"
#include "manipulator_kinematics/joint_state.hpp"
#include "manipulator_kinematics/robot_config.hpp"

/**
 * @class ForwardKinematics
 * @brief Performs forward kinematics calculations for a robotic manipulator.
 *
 * This class provides the functionality to compute the end-effector state of a
 * robot based on its joint states and robot configuration. It utilizes the
 * Eigen library for matrix and vector operations to perform the kinematic
 * calculations.
 */
class ForwardKinematics
{
public:
  /**
   * @brief Constructs a ForwardKinematics object with the given robot
   * configuration.
   * @param robot_config The configuration of the robot, including link lengths
   * and joint limits.
   */
  explicit ForwardKinematics(const RobotConfig& robot_config) : robot_config_(robot_config)
  {
  }

  /**
   * @brief Computes the end-effector state given the current joint states.
   * @param joint_state The current state of the robot's joints, including
   * positions, velocities, and accelerations.
   * @return The computed end-effector state, including position and
   * orientation.
   */
  [[nodiscard]] EEState computeEEState(const JointState& joint_state) const;

  /**
   * @brief Retrieves the robot configuration used by this instance.
   * @return A constant reference to the robot's configuration.
   */
  [[nodiscard]] const RobotConfig& getRobotConfig() const
  {
    return robot_config_;
  }

  /**
   * @brief Returns the size of the spatial vector.
   */
  static constexpr int spatial_vector_size = 6;

private:
  RobotConfig robot_config_;  ///< Robot configuration
  //   const RobotConfig& robot_config_;  ///< Robot configuration //FIXME:
  //   should this be a reference?

  /**
   * @brief Converts an angular velocity vector to a skew-symmetric matrix.
   * @param omega The angular velocity vector.
   * @return The corresponding skew-symmetric matrix.
   */
  [[nodiscard]] Eigen::Matrix3d omegaHat(const Eigen::Vector3d& omega) const;

  /**
   * @brief Computes the Rodrigues' rotation formula.
   * @param hatOmega The skew-symmetric matrix of angular velocity.
   * @param theta The rotation angle in radians.
   * @return The rotation matrix computed using Rodrigues' formula.
   */
  [[nodiscard]] Eigen::Matrix3d rodriguesRotation(const Eigen::Matrix3d& hatOmega, double theta) const;

  /**
   * @brief Calculates the translation vector for a given screw motion.
   * @param R The rotation matrix.
   * @param hatOmega The skew-symmetric matrix of angular velocity.
   * @param v The linear velocity vector.
   * @param theta The rotation angle in radians.
   * @return The translation vector for the screw motion.
   */
  [[nodiscard]] Eigen::Vector3d calculateTranslation(const Eigen::Matrix3d& R, const Eigen::Matrix3d& hatOmega,
                                                     const Eigen::Vector3d& v, double theta) const;

  /**
   * @brief Computes the exponential of a twist.
   * @param screwAxis The screw axis represented as a 6D vector.
   * @param theta The magnitude of the twist (rotation in radians for revolute
   * joints, translation for prismatic joints).
   * @return The homogeneous transformation matrix representing the screw
   * motion.
   */
  [[nodiscard]] Eigen::Matrix4d expTwist(const Eigen::VectorXd& screwAxis, double theta) const;

  /**
   * @brief Performs the forward kinematics calculation.
   * @param screwAxes The matrix of screw axes for each joint in the robot.
   * @param thetas The vector of joint angles/positions.
   * @return The homogeneous transformation matrix of the robot's end-effector.
   */
  [[nodiscard]] Eigen::Matrix4d computeFK(const Eigen::MatrixXd& screwAxes, const Eigen::VectorXd& thetas) const;

  /**
   * @brief Computes the screw axis for a revolute joint.
   * @param w The unit vector along the axis of rotation.
   * @param q A point through which the axis of rotation passes.
   * @return The screw axis as a 6D vector.
   */
  [[nodiscard]] Eigen::VectorXd getRevoluteScrewAxis(const Eigen::Vector3d& w, const Eigen::Vector3d& q) const;

  /**
   * @brief Gathers the screw axes for all the joints of the robot.
   * @return A matrix where each column represents the screw axis of a joint.
   */
  [[nodiscard]] Eigen::MatrixXd getScrewAxes() const;
};

#endif  // MANIPULATOR_KINEMATICS_FORWARD_KINEMATICS_H