# Test targets
add_executable(config_test robot_config_test.cpp)
add_executable(fk_test fk_test.cpp)
add_executable(ws_test ws_test.cpp)
add_executable(ik_test ik_test.cpp)

# Link tests with GoogleTest and the project library
foreach(test_target config_test fk_test ws_test ik_test)
  target_link_libraries(${test_target} gtest_main manipulator_kinematics)
  gtest_discover_tests(${test_target})
endforeach()