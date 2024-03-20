#include <gtest/gtest.h>  // cppcheck-suppress legal/copyright

#include <cmath>

#include "manipulator_kinematics/circular_workspace.hpp"
#include "manipulator_kinematics/ee_state.hpp"
#include "manipulator_kinematics/forward_kinematics.hpp"
#include "manipulator_kinematics/joint_state.hpp"
#include "manipulator_kinematics/robot_config.hpp"

// Test ficture for FK tests
class WSTest : public ::testing::Test {
 public:
  void SetUp() override {
    try {
      // Load the robot configuration
      RobotConfig config("./data/test_robot_config_2R.yaml");
      // Create a robot object and joint state
      fk = std::make_unique<ForwardKinematics>(config);
    } catch (std::exception& e) {
      std::cerr << e.what() << std::endl;
    }
  }

  std::unique_ptr<ForwardKinematics> fk;
};

// Test case for forward kinematics
TEST_F(WSTest, InWorkspace) {
  // Define a vector of joint angles
  std::vector<double> joint_angles = {0, 0};
  const auto& robot_config = fk->getRobotConfig();
  JointState joint_state(robot_config);
  joint_state.setJointAngles(joint_angles);

  // Compute forward kinematics
  EEState ee_state = fk->computeEEState(joint_state);

  // Create a circular workspace
  CircularWorkspace circ_ws(3.0, 8.0, 0.0);
  // Workspace ws(circ_ws);

  // Check if the end effector is within the workspace
  bool is_ee_in_workspace = circ_ws.isWithinWorkspace(ee_state);
  EXPECT_EQ(is_ee_in_workspace, true);
}

TEST_F(WSTest, OutOfWorkspace) {
  // Define a vector of joint angles
  std::vector<double> joint_angles = {0, M_PI / 2};
  const auto& robot_config = fk->getRobotConfig();
  JointState joint_state(robot_config);
  joint_state.setJointAngles(joint_angles);

  // Compute forward kinematics
  EEState ee_state = fk->computeEEState(joint_state);

  // Create a circular workspace
  CircularWorkspace circ_ws(2.0, 10.0, 10.0);
  // Workspace ws(circ_ws);

  // Check if the end effector is within the workspace
  bool is_ee_in_workspace = circ_ws.isWithinWorkspace(ee_state);
  EXPECT_EQ(is_ee_in_workspace, false);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
