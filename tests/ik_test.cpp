#include <gtest/gtest.h>
#include <cmath>
#include "manipulator_kinematics/ee_state.h"
#include "manipulator_kinematics/joint_state.h"
#include "manipulator_kinematics/inverse_kinematics.h"
#include "manipulator_kinematics/robot_config.h"

// Test ficture for IK tests
class IKTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    try
    {
      // Load the robot configuration
      RobotConfig config("../tests/data/test_robot_config_3R.yaml");
      // Create a robot object and joint state

      ik = std::make_unique<InverseKinematics>(config);
    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
  }

  std::unique_ptr<InverseKinematics> ik;
};

// Test case for inverse kinematics
TEST_F(IKTest, Test1)
{
  // Define a vector of joint angles
  EEState ee_state({ .x = 5.0, .y = -10.0, .thetaP = -M_PI / 2 });

  // Compute inverse kinematics
  JointState joint_state = ik->computeJointState(ee_state);

  // Expected values
  std::vector<double> expectedJointAngles = { -M_PI / 2, M_PI / 2, -M_PI / 2 };

  // Check if the computed values are within the tolerance
  for (int i = 0; i < joint_state.getJointAngles().size(); i++)
  {
    EXPECT_NEAR(joint_state.getJointAngle(i), expectedJointAngles[i], 1e-5);
  }
}

// TODO: Add more test cases

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
