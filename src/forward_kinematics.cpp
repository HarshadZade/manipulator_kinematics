#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "manipulator_kinematics/forward_kinematics.h"

EEState ForwardKinematics::computeEEState(const JointState& joint_state) const
{
  // Convert joint angles to Eigen vector // TODO: make this a helper function
  const auto& joint_angles = joint_state.getJointAngles();
  Eigen::VectorXd thetas(joint_angles.size());
  for (int i = 0; i < joint_angles.size(); i++)
  {
    thetas(i) = joint_angles.at(i);
  }

  // Compute forward kinematics
  Eigen::Matrix4d T = computeFK(getScrewAxes(), thetas);

  return { .x = T(0, 3), .y = T(1, 3), .thetaP = thetas.sum() };
}

// Forward kinematics function
const Eigen::Matrix4d ForwardKinematics::computeFK(const Eigen::MatrixXd& screwAxes,
                                                   const Eigen::VectorXd& thetas) const
{
  // Initialize the transformation matrix
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  // Compute the exponential of each twist
  for (int i = 0; i < thetas.size(); i++)
  {
    T = T * expTwist(screwAxes.col(i), thetas(i));
  }

  // define
  // T0 = [1 0 0 l1+l2+l3;
  //       0 1 0 0;
  //       0 0 1 0;
  //       0 0 0 1]
  // multiply T0 with T to get the final transformation matrix
  Eigen::Matrix4d T0 = Eigen::Matrix4d::Identity();
  double len = 0.0;

  for (int i = 0; i < robot_config_.getNumLinks(); i++)
  {
    len += robot_config_.getLinks().at(i).length;
  }

  T0(0, 3) = len;
  T = T * T0;

  return T;
}

// Function to convert an angular velocity vector into a skew-symmetric matrix
const Eigen::Matrix3d ForwardKinematics::omegaHat(const Eigen::Vector3d& omega) const
{
  Eigen::Matrix3d hatOmega;
  hatOmega << 0, -omega.z(), omega.y(), omega.z(), 0, -omega.x(), -omega.y(),  // cppcheck-suppress constStatement
      omega.x(), 0;                                                            // cppcheck-suppress constStatement
  return hatOmega;
}

// Function to compute the Rodrigues' rotation formula
const Eigen::Matrix3d ForwardKinematics::rodriguesRotation(const Eigen::Matrix3d& hatOmega, double theta) const
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + sin(theta) * hatOmega + (1 - cos(theta)) * hatOmega * hatOmega;
  return R;
}

// Function to calculate the translation vector for a screw motion
const Eigen::Vector3d ForwardKinematics::calculateTranslation(const Eigen::Matrix3d& R, const Eigen::Matrix3d& hatOmega,
                                                              const Eigen::Vector3d& v, double theta) const
{
  Eigen::Vector3d p =
      (Eigen::Matrix3d::Identity() * theta + (1 - cos(theta)) * hatOmega + (theta - sin(theta)) * hatOmega * hatOmega) *
      v;
  return p;
}

// Main function to compute the exponential of a twist
const Eigen::Matrix4d ForwardKinematics::expTwist(const Eigen::VectorXd& screwAxis, double theta) const
{
  // Convert angular part to skew-symmetric matrix
  Eigen::Matrix3d hatOmega = omegaHat(screwAxis.head(3));

  // Compute Rodrigues' rotation formula
  Eigen::Matrix3d R = rodriguesRotation(hatOmega, theta);

  // Calculate translation vector for screw motion
  Eigen::Vector3d v(screwAxis(3), screwAxis(4), screwAxis(5));
  Eigen::Vector3d p = calculateTranslation(R, hatOmega, v, theta);

  // Assemble into homogeneous transformation matrix
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = p;
  return T;
}

// Get screw axis for a revolute joint
const Eigen::VectorXd ForwardKinematics::getRevoluteScrewAxis(const Eigen::Vector3d& w, const Eigen::Vector3d& q) const
{
  Eigen::VectorXd screwAxis(6);  // cppcheck-suppress constStatement
  screwAxis << w, -w.cross(q);   // cppcheck-suppress constStatement
  return screwAxis;
}

// Function for screwAxes for the robot
const Eigen::MatrixXd ForwardKinematics::getScrewAxes() const
{
  // get number of links from robot_config_ and use it to initialize screwAxes
  int num_links = robot_config_.getNumLinks();
  Eigen::MatrixXd screwAxes(6, num_links);

  Eigen::Vector3d w(0, 0, 1);  // Assume all rotations are about the z-axis
  Eigen::Vector3d q(0, 0, 0);  // Starting position of the first link

  for (int i = 0; i < num_links; i++)
  {
    // For the first link, the position is (0,0,0)
    screwAxes.col(i) = getRevoluteScrewAxis(w, q);
    if (i < num_links - 1)
    {
      q(0) += robot_config_.getLinks().at(i).length;
    }
  }
  return screwAxes;
}