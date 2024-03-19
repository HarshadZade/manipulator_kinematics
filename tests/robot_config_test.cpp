#include "manipulator_kinematics/robot_config.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>

auto test_config_path = "../tests/data/test_robot_config_2R.yaml";

// Test for successful loading of configuration file
TEST(RobotConfigTest, LoadsConfigurationSuccessfully) {
  RobotConfig robotConfig(test_config_path);
  const auto& links = robotConfig.getLinks();
  const auto& joints = robotConfig.getJoints();
  const auto num_links = robotConfig.getNumLinks();

  // Verification for links
  ASSERT_EQ(links.size(), 2);
  EXPECT_EQ(links[0].id, 0);
  EXPECT_DOUBLE_EQ(links[1].length, 5.0);

  // Verification for joints
  ASSERT_EQ(joints.size(), 2);
  EXPECT_EQ(joints[0].id, 0);
  EXPECT_EQ(joints[1].theta_min, -M_PI);
  EXPECT_EQ(joints[1].theta_max, M_PI);

  // Verification for number of links
  EXPECT_EQ(num_links, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
