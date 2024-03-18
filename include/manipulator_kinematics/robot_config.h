#ifndef MANIPULATOR_KINEMATICS_ROBOT_CONFIG_H
#define MANIPULATOR_KINEMATICS_ROBOT_CONFIG_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

// Structure to represent a single link in the robot arm
struct Link
{
  int id;
  double length;
};

// Structure to represent a single joint in the robot arm
struct Joint
{
  int id;
  double theta_min;
  double theta_max;
};

class RobotConfig
{
public:
  explicit RobotConfig(const std::string& config_path);
  void loadConfiguration(const std::string& config_path);
  [[nodiscard]] const std::vector<Link>& getLinks() const noexcept;
  [[nodiscard]] const std::vector<Joint>& getJoints() const noexcept;
  [[nodiscard]] const int getNumLinks() const noexcept;

private:
  std::vector<Link> links_;
  std::vector<Joint> joints_;
  int num_links_;

  void parseLink(const YAML::Node& node);
  void parseJoint(const YAML::Node& node);
  void setNumLinks(const YAML::Node& node);
};

#endif  // MANIPULATOR_KINEMATICS_ROBOT_CONFIG