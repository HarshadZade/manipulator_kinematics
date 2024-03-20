
---

# manipulator_kinematics

## Description
This project implements the forward and inverse kinematics of a 3R planar manipulator using Screw Theory for forward kinematics and a Geometric approach for inverse kinematics, without relying on external kinematics libraries. It's designed to demonstrate clean, reusable, extendable, and maintainable coding practices in modern C++.

## Authors
- [Author 1] Harshad Zade

## Getting Started

### Software Architecture
The software architecture is designed to be modular and extensible. The project is divided into the following modules:
![Software Architecture](design/software_architecture.png)

### Note (Assumptions and Conventions)
- All dimensions are in SI units (meters, radians, etc.).
- The orientation of the end-effector is represented by the sum of the joint angles.
- Joints allow full rotation within their defined limits without considering real-world physical constraints like collisions.

### Dependencies
- **C++17 Compiler**: Ensure a modern compiler (e.g., GCC or Clang) is installed.
- **CMake**: For building the project.
- **Google Test**: For unit testing.
- **Eigen**: For matrix operations used in kinematics calculations.
- **Doxygen**: For generating documentation.

### Installing Dependencies

#### Eigen
Eigen is a header-only library, simplifying installation. On Ubuntu:
```bash
sudo apt-get install libeigen3-dev
```

#### Doxygen
Install Doxygen for documentation generation:
```bash
sudo apt-get install doxygen
```

### Building the Project
To build the project, run the following commands in the project root directory:
```bash
cmake -S ./ -B build/
cmake --build build/
```

For building with code coverage:
```bash
cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -D ENABLE_TESTING=TRUE -S ./ -B build/ 
cmake --build build/ --target all test_coverage
```
This generates a index.html page in the `build/test_coverage` sub-directory that can be viewed locally in a web browser.
```bash
xdg-open build/test_coverage/index.html
```

### Running Tests
To run the unit tests, navigate to the `build/` directory and use CTest (installed with CMake):
```bash
cd build/
ctest
ctest -V # For verbose output
```

## Documentation

### Generating Doxygen Documentation
In the project root directory, run the following command:

```bash
mkdir docs
cmake --build build/ --target docs
```

This generates a index.html page in the `docs/html` sub-directory that can be viewed locally in a web browser.
```bash
xdg-open docs/html/index.html
```

## Code Quality Tools

### Clang-format
To enforce Google's C++ style guide:
```bash
clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^./build/")
```

### Cppcheck and Cpplint
Install cppcheck and cpplint
```bash
sudo apt-get install cppcheck
pip install cpplint
```

Run static code analysis and linting:
```bash
cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude --inline-suppr $(find . -name *.cpp | grep -vE -e "^./build/") &> temp/cppcheck

cpplint $(find . -name *.cpp | grep -vE -e "^./build/") &> temp/cpplint
```

## Using Address Sanitizer
The address sanitizer is a runtime tool that detects memory errors in C/C++ programs. It has been added to the CMake build system when gtest is enabled. 

## References
- **Eigen**: [Eigen's Official Documentation](http://eigen.tuxfamily.org/dox/)
- **Google Test**: [Google Test GitHub Repository](https://github.com/google/googletest)
- **Cppcheck**: [Cppcheck GitHub Repository](https://github.com/danmar/cppcheck)
- **Cpplint**: [Cpplint Tool](https://github.com/cpplint/cpplint)
- **Doxygen**: [Doxygen Official Site](https://www.doxygen.nl/index.html)
- **Modern Robotics**: [Modern Robotics: Mechanics, Planning, and Control](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- **MLS**: [A Mathematical Introduction to Robotic Manipulation](http://www.cds.caltech.edu/~murray/mlswiki/index.php/Main_Page)

---