# Test targets
add_executable(config_test robot_config_test.cpp)
add_executable(fk_test fk_test.cpp)
add_executable(ws_test ws_test.cpp)
add_executable(ik_test ik_test.cpp)

# Link tests with GoogleTest and the project library
foreach(test_target config_test fk_test ws_test ik_test)
  target_link_libraries(${test_target} gtest_main manipulator_kinematics)
  include(GoogleTest)
  gtest_discover_tests(${test_target})
endforeach()

# Add -fsanitize=address if testing enabled
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(config_test PRIVATE -fsanitize=address)
    target_link_options(config_test PRIVATE -fsanitize=address)
    target_compile_options(fk_test PRIVATE -fsanitize=address)
    target_link_options(fk_test PRIVATE -fsanitize=address)
    target_compile_options(ws_test PRIVATE -fsanitize=address)
    target_link_options(ws_test PRIVATE -fsanitize=address)
    target_compile_options(ik_test PRIVATE -fsanitize=address)
    target_link_options(ik_test PRIVATE -fsanitize=address)
    # Repeat for other test executables as needed
endif()
