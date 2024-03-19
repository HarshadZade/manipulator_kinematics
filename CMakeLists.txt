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
# Doxygen Support check if Doxygen is installed
# ref: https://cmake.org/cmake/help/latest/module/FindDoxygen.html
#
find_package(Doxygen)
if ( DOXYGEN_FOUND )
  set( DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_EXTRACT_ALL          YES )
  set( DOXYGEN_CLASS_DIAGRAMS       YES )
  set( DOXYGEN_HIDE_UNDOC_RELATIONS NO )
  set( DOXYGEN_HAVE_DOT             YES )
  set( DOXYGEN_CLASS_GRAPH          YES )
  set( DOXYGEN_CALL_GRAPH           YES )
  set( DOXYGEN_CALLER_GRAPH         YES )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_BUILTIN_STL_SUPPORT  YES )
  set( DOXYGEN_EXTRACT_PRIVATE      YES )
  set( DOXYGEN_EXTRACT_PACKAGE      YES )
  set( DOXYGEN_EXTRACT_STATIC       YES )
  set( DOXYGEN_EXTRACT_LOCALMETHODS YES )
  set( DOXYGEN_UML_LOOK             YES )
  set( DOXYGEN_UML_LIMIT_NUM_FIELDS 50 )
  set( DOXYGEN_TEMPLATE_RELATIONS   YES )
  set( DOXYGEN_DOT_GRAPH_MAX_NODES  100 )
  set( DOXYGEN_MAX_DOT_GRAPH_DEPTH  0 )
  set( DOXYGEN_DOT_TRANSPARENT      YES )
  set( DOXYGEN_MARKDOWN_SUPPORT       YES)
  set( DOXYGEN_USE_MDFILE_AS_MAINPAGE "README.md")
else()
  message( FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation" )
endif()

# create a target to build documentation
doxygen_add_docs(docs           # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/tests
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/README.md
  )

#
# Include directories
#
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(manipulator_kinematics STATIC 
  src/robot_config.cpp
  src/forward_kinematics.cpp
  src/workspace.cpp
  src/inverse_kinematics.cpp
)

#
# Add Eigen3
#
find_package(Eigen3 3.3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

#
# yaml-cpp
#
find_package(yaml-cpp REQUIRED)
target_link_libraries(manipulator_kinematics PRIVATE yaml-cpp Eigen3::Eigen)

#
# Add the test executable
#
add_executable(config_test
  tests/robot_config_test.cpp
)
target_link_libraries(config_test PRIVATE gtest_main manipulator_kinematics)

add_executable(fk_test
  tests/fk_test.cpp
)
target_link_libraries(fk_test PRIVATE gtest_main manipulator_kinematics)

add_executable(ws_test
  tests/ws_test.cpp
)
target_link_libraries(ws_test PRIVATE gtest_main manipulator_kinematics)

add_executable(ik_test
  tests/ik_test.cpp
)
target_link_libraries(ik_test PRIVATE gtest_main manipulator_kinematics)

#
# Enable testing and define the tests
#
enable_testing()
include(GoogleTest)
gtest_discover_tests(config_test)
gtest_discover_tests(fk_test)
gtest_discover_tests(ws_test)
gtest_discover_tests(ik_test)
