#ifndef MANIPULATOR_KINEMATICS_JOINT_STATE_H
#define MANIPULATOR_KINEMATICS_JOINT_STATE_H

#include <vector>
#include "manipulator_kinematics/robot_config.h"

class JointState
{
public:
  /**
   * @brief Constructor that initializes the JointState with RobotConfig.
   * @param config_path Path to the YAML configuration file.
   */
  explicit JointState(const RobotConfig& config) : robot_config_(config), joint_angles_(config.getNumLinks(), 0.0)
  {
  }

  /**
   * @brief Sets the joint angles of the robot arm.
   * @param
   */
  void setJointAngle(const int joint_id, const double angle)
  {
    // Check if the angle is within the joint limits
    const Joint& joint = robot_config_.getJoints().at(joint_id);
    if (angle < joint.theta_min || angle > joint.theta_max)
    {
      std::ostringstream msg;
      msg << "Joint angle is out of range: Joint ID = " << joint_id << ", Angle = " << angle;
      throw std::out_of_range(msg.str());
    }
    joint_angles_.at(joint_id) = angle;
  }

  // Set all joint angles
  void setJointAngles(const std::vector<double>& angles)
  {
    // Check if the angles are within the joint limits
    for (int i = 0; i < angles.size(); i++)
    {
      const Joint& joint = config_.getJoints().at(i);
      if (angles.at(i) < joint.theta_min || angles.at(i) > joint.theta_max)
      {
        std::ostringstream msg;
        msg << "Joint angle is out of range: Joint ID = " << i << ", Angle = " << angles.at(i);
        throw std::out_of_range(msg.str());
      }
    }
    joint_angles_ = angles;
  }

  // Get joint angle
  [[nodiscard]] double getJointAngle(const int joint_id) const
  {
    return joint_angles_.at(joint_id);
  }

  // Get all joint angles
  [[nodiscard]] const std::vector<double>& getJointAngles() const noexcept
  {
    return joint_angles_;
  }

private:
  const RobotConfig& robot_config_;  ///< Reference to the RobotConfig object.
  //   RobotConfig robot_config_;     ///< RobotConfig object.
  std::vector<double> joint_angles_;  ///< Vector of joint angles in the robot arm.
};
#endif  // MANIPULATOR_KINEMATICS_JOINT_STATE_H