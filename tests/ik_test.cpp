#include <gtest/gtest.h>

#include <cmath>

#include "manipulator_kinematics/ee_state.hpp"
#include "manipulator_kinematics/inverse_kinematics.hpp"
#include "manipulator_kinematics/joint_state.hpp"
#include "manipulator_kinematics/robot_config.hpp"

// Test ficture for IK tests
class IKTest : public ::testing::Test {
 public:
  void SetUp() override {
    try {
      // Load the robot configuration
      RobotConfig config("./data/test_robot_config_3R.yaml");
      // Create a robot object and joint state

      ik = std::make_unique<InverseKinematics>(config);
    } catch (std::exception& e) {
      std::cerr << e.what() << std::endl;
    }
  }

  std::unique_ptr<InverseKinematics> ik;
};

// Test case for inverse kinematics
TEST_F(IKTest, Test1) {
  // Define a vector of joint angles
  EEState ee_state({.x = -10.0, .y = 5.0, .thetaP = M_PI});

  // Compute inverse kinematics
  std::vector<JointState> joint_state = ik->computeJointState(ee_state);

  // Expected values
  std::vector<double> expected_joint_angles_1 = {M_PI / 2, M_PI / 2, 0};
  std::vector<double> expected_joint_angles_2 = {M_PI, -M_PI / 2, M_PI / 2};

  // Check if the computed values are within the tolerance (for both solutions)
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(joint_state[0].getJointAngles()[i], expected_joint_angles_1[i],
                1e-5);
    EXPECT_NEAR(joint_state[1].getJointAngles()[i], expected_joint_angles_2[i],
                1e-5);
  }
}

// TODO: Add more test cases

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
