#include <gtest/gtest.h>

#include <cmath>

#include "manipulator_kinematics/ee_state.h"
#include "manipulator_kinematics/joint_state.h"
#include "manipulator_kinematics/forward_kinematics.h"
#include "manipulator_kinematics/robot_config.h"

// Test ficture for FK tests
class FKTest : public ::testing::Test
{
public:  // cppcheck-suppress unusedFunction
  void SetUp() override
  {
    try
    {
      // Load the robot configuration
      RobotConfig config("../tests/data/test_robot_config_3R.yaml");
      // Create a robot object and joint state

      fk = std::make_unique<ForwardKinematics>(config);
    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
  }

  std::unique_ptr<ForwardKinematics> fk;
};

// Test case for forward kinematics
TEST_F(FKTest, Test1)
{
  // Define a vector of joint angles
  std::vector<double> joint_angles = { 0, M_PI / 2, 0 };

  const auto& robot_config = fk->getRobotConfig();
  JointState joint_state(robot_config);
  joint_state.setJointAngles(joint_angles);

  // Compute forward kinematics
  EEState ee_state = fk->computeEEState(joint_state);

  // Expected values
  double expectedX = 5.0;
  double expectedY = 10.0;
  double expectedThetaP = M_PI / 2;

  // Check if the computed values are within the tolerance
  EXPECT_NEAR(ee_state.x, expectedX, 1e-5);
  EXPECT_NEAR(ee_state.y, expectedY, 1e-5);
  EXPECT_NEAR(ee_state.thetaP, expectedThetaP, 1e-5);
}

// TODO: Add more test cases

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
