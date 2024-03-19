#include <iostream>
#include <fstream>
#include <cmath>
#include "manipulator_kinematics/robot_config.h"

// Constructor
RobotConfig::RobotConfig(const std::string& config_path)
{
  loadConfiguration(config_path);
}

// Load and parse the YAML configuration file
void RobotConfig::loadConfiguration(const std::string& config_path)
{
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(config_path);
    std::cout << "Loaded configuration file successfully" << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "Error loading configuration file: " << e.what() << std::endl;
    throw;
  }
  try
  {
    for (const auto& link : config["robot"]["links"])
    {
      parseLink(link);
    }
    for (const auto& joint : config["robot"]["joints"])
    {
      parseJoint(joint);
    }
    setNumLinks(config["robot"]);
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "Error parsing either links or joints from the configuration file: " << e.what() << std::endl;
    throw;
  }
}

// Accessor for links
const std::vector<Link>& RobotConfig::getLinks() const noexcept
{
  return links_;
}

// Accessor for joints
const std::vector<Joint>& RobotConfig::getJoints() const noexcept
{
  return joints_;
}

// Accessor for number of links
int RobotConfig::getNumLinks() const noexcept
{
  return num_links_;
}

// Parse a single link from the YAML node
void RobotConfig::parseLink(const YAML::Node& node)
{
  Link link;
  link.id = node["id"].as<int>();
  link.length = node["length"].as<double>();
  links_.push_back(link);
}

// Parse a single joint from the YAML node
void RobotConfig::parseJoint(const YAML::Node& node)
{
  Joint joint;
  joint.id = node["id"].as<int>();
  double theta_min_degrees = node["theta_min"].as<double>();
  joint.theta_min = theta_min_degrees * (M_PI / 180.0);
  double theta_max_degrees = node["theta_max"].as<double>();
  joint.theta_max = theta_max_degrees * (M_PI / 180.0);
  joints_.push_back(joint);
}

// Set the number of links in the robot
void RobotConfig::setNumLinks(const YAML::Node& node)
{
  num_links_ = node["num_links"].as<int>();
}