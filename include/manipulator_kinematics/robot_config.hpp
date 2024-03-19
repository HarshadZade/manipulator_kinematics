#ifndef MANIPULATOR_KINEMATICS_ROBOT_CONFIG_H
#define MANIPULATOR_KINEMATICS_ROBOT_CONFIG_H

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

/**
 * @brief Structure to represent a single link in the robot arm.
 */
struct Link
{
  int id;         ///< Link identifier.
  double length;  ///< Length of the link.
};

/**
 * @brief Structure to represent a single joint in the robot arm.
 */
struct Joint
{
  int id;            ///< Joint identifier.
  double theta_min;  ///< Minimum theta value for the joint.
  double theta_max;  ///< Maximum theta value for the joint.
};

/**
 * @brief Class to manage the configuration of a robotic manipulator.
 * This class provides functionalities to load and access configurations of
 * a robotic manipulator from a YAML file.
 */
class RobotConfig
{
public:
  /**
   * @brief Constructor that initializes the RobotConfig with a configuration
   * file.
   * @param config_path Path to the YAML configuration file.
   */
  explicit RobotConfig(const std::string& config_path);

  /**
   * @brief Loads the robot configuration from the specified YAML file.
   * @param config_path Path to the YAML configuration file.
   */
  void loadConfiguration(const std::string& config_path);

  /**
   * @brief Gets the links of the robot arm.
   * @return A constant reference to a vector of Link structures.
   */
  [[nodiscard]] const std::vector<Link>& getLinks() const noexcept;

  /**
   * @brief Gets the joints of the robot arm.
   * @return A constant reference to a vector of Joint structures.
   */
  [[nodiscard]] const std::vector<Joint>& getJoints() const noexcept;

  /**
   * @brief Gets the number of links in the robot arm.
   * @return The number of links as an integer.
   */
  [[nodiscard]] int getNumLinks() const noexcept;

private:
  std::vector<Link> links_;    ///< Vector of links in the robot arm.
  std::vector<Joint> joints_;  ///< Vector of joints in the robot arm.
  int num_links_;              ///< Number of links in the robot arm.

  /**
   * @brief Parses a link from a YAML node and adds it to the links vector.
   * @param node The YAML node containing the link data.
   */
  void parseLink(const YAML::Node& node);

  /**
   * @brief Parses a joint from a YAML node and adds it to the joints vector.
   * @param node The YAML node containing the joint data.
   */
  void parseJoint(const YAML::Node& node);

  /**
   * @brief Sets the number of links in the robot arm from a YAML node.
   * @param node The YAML node containing the number of links.
   */
  void setNumLinks(const YAML::Node& node);
};

#endif  // MANIPULATOR_KINEMATICS_ROBOT_CONFIG_H
