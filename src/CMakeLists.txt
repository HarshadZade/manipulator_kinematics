# Add library target
add_library(manipulator_kinematics STATIC 
  robot_config.cpp
  forward_kinematics.cpp
  circular_workspace.cpp
  inverse_kinematics.cpp
  joint_state.cpp
)

# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# Include directories
target_include_directories(manipulator_kinematics PUBLIC ${PROJECT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})

# Link libraries
target_link_libraries(manipulator_kinematics PRIVATE yaml-cpp Eigen3::Eigen)
