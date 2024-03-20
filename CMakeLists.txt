cmake_minimum_required(VERSION 3.14)
project(manipulator_kinematics)

#
# Set C++ standard
#
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Add -fsanitize=address when testing is enabled
if(ENABLE_TESTING)
    add_compile_options(-fsanitize=address)
    add_link_options(-fsanitize=address)
endif()

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

# Enables testing for this directory and below
enable_testing()
include(GoogleTest)

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

if(ENABLE_TESTING)

#
# Install test data
#
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests/data
        DESTINATION ${CMAKE_INSTALL_PREFIX}/tests)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/tests/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/tests)

#
# Set up code coverage generation.
# Must compile with debug, e.g,
#   cmake -S ./ -B build/ -D CMAKE_BUILD_TYPE=Debug
#
option(WANT_COVERAGE "this option enable coverage" ON)
if(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  set(WANT_COVERAGE OFF)
endif()

if(WANT_COVERAGE)
  message("Enabling coverage")
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules")
  include(CodeCoverage)
  # The following line come from cmake_modules/CodeCoverage.cmake
  append_coverage_compiler_flags() # will add "--coverage" compilation flag
  # Exclude certain directories from coverage analysis
  set(COVERAGE_EXCLUDES
    ${PROJECT_BINARY_DIR}
    )
endif()

# create targets for building code coverage reports
if(WANT_COVERAGE)
  # unit test coverage:
  setup_target_for_coverage_lcov ( # use lcov to generate coverage report
    NAME test_coverage          # target name
    # EXECUTABLE ctest || true    # what to run insdie the build directory?
    #                             #   '|| true' --> continue coverage even if ctest failed
    EXECUTABLE ctest     # what to run insdie the build directory?
    EXCLUDE
    #   "app/main.cpp"     # Unit test does not run app, so don't analyze it
      "*gtest*"          # Don't analyze googleTest code
      "/usr/include/*"   # Don't analyze system headers
      "/usr/lib/gcc/x86_64-linux-gnu/9/include/*"   # Don't analyze system headers
    )

  # code coverage for the app (shell-app):
  # Use gcovr insetad of lcov if you want to see branch coverage as well
#   setup_target_for_coverage_gcovr_html ( # use gcovr to generate coverage report
#     NAME app_coverage          # target name
#     EXECUTABLE shell-app       # what to run insdie the build directory?
#     EXCLUDE
#     "test/test.cpp"    # App does not run unit test, so don't analyze it
#     "test/main.cpp"    # App does not run unit test, so don't analyze it
#     )
endif()
endif()

#
# Add subdirectories
#
add_subdirectory(src)
if(ENABLE_TESTING)
  add_subdirectory(tests)
endif()