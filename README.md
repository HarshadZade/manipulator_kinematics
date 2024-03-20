# manipulator_kinematics

## Description
This project is an implementation of the forward and inverse kinematics of a 3R planar manipulator. The forward kinematics is implemented using the Screw Theory and the inverse kinematics is implemented using the Geometric method.

## Software Architecture
![Software Architecture](design/software_architecture.png)

## Doxygen Documentation
To generate the doxygen documentation:
Install doxygen
```bash
sudo apt-get install doxygen

# Generate the documentation
# In the project root directory, run the following command:
mkdir docs
cmake --build build/ --target docs

# To view
xdg-open docs/html/index.html
```

## run clang-format
```bash
# In the project root directory, run the following command:
clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^./build/")
```

## cppcheck and cpplint
Install cppcheck and cpplint
```bash
sudo apt-get install cppcheck
pip install cpplint

# Run cpplint
cpplint --filter="-legal/copyright","-build/include_subdir" $( find . -name *.cpp | grep -vE -e "^./build/" ) &> temp/cpplint

# Run cppcheck
cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude --inline-suppr $( find . -name *.cpp | grep -vE -e "^./build/" ) &> temp/cppcheck
```

<!-- test build and code cov -->
cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -D ENABLE_TESTING=TRUE -S ./ -B build/ 
cmake --build build/
cmake --build build/ --clean-first --target all test_coverage


<!-- standard bulding wo testing -->
cmake -S ./ -B build/
cmake --build build/
cmake --build build/ --clean-first