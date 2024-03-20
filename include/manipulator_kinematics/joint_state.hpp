#ifndef MANIPULATOR_KINEMATICS_JOINT_STATE_H
#define MANIPULATOR_KINEMATICS_JOINT_STATE_H

#include <vector>

#include "manipulator_kinematics/robot_config.hpp"

/**
 * @class JointState
 * @brief Represents the state of the joints in a robotic manipulator.
 *
 * This class manages the joint angles of a robot manipulator, ensuring they
 * stay within defined limits based on the robot's configuration.
 */
class JointState {
 public:
  /**
   * @brief Constructs a JointState object with the provided RobotConfig.
   * @param config The robot configuration specifying the number and limits of
   * joints.
   */
  explicit JointState(const RobotConfig& config)
      : robot_config_(config), joint_angles_(config.getNumLinks(), 0.0) {}

  /**
   * @brief Sets the angles of all joints, ensuring each is within its limits.
   * Throws an std::out_of_range exception if any specified angle is out of its
   * corresponding joint's allowed range.
   * @param angles A vector of angles for the joints in radians.
   * @throw std::out_of_range if any angle is outside its joint's limits.
   */
  void setJointAngles(const std::vector<double>& angles);

  /**
   * @brief Retrieves the angles of all joints.
   * @return A constant reference to a vector of all joint angles in radians.
   */
  [[nodiscard]] const std::vector<double>& getJointAngles() const noexcept;

 private:
  const RobotConfig& robot_config_;  ///< Reference to the RobotConfig object.
  std::vector<double>
      joint_angles_;  ///< Vector of joint angles in the robot arm.
};
#endif  // MANIPULATOR_KINEMATICS_JOINT_STATE_H