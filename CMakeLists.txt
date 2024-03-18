cmake_minimum_required(VERSION 3.14)
project(manipulator_kinematics)

#
# Set C++ standard
#
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#
# Fetch content for Google Test
#
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.11.0
)

#
# For simplicity, disable installing Google Test to avoid polluting the global cmake install space
#
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

#
# Include directories
#
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(manipulator_kinematics STATIC 
  src/robot_config.cpp
)

#
# yaml-cpp
#
find_package(yaml-cpp REQUIRED)
target_link_libraries(manipulator_kinematics PRIVATE yaml-cpp)

#
# Add the test executable
#
add_executable(config_test
  tests/robot_config_test.cpp
)
target_link_libraries(config_test PRIVATE gtest_main manipulator_kinematics)

#
# Enable testing and define the tests
#
enable_testing()
include(GoogleTest)
gtest_discover_tests(config_test)