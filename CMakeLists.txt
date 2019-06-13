# Minimum CMake Version Required
cmake_minimum_required(VERSION 2.6)
project(EKF)

# Use C++11
set(CMAKE_CXX_STANDARD 11)

# Compiler Flags
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wall -Werror")

# Extra Include Files
include_directories(
    inc/
)

# Source Files
set(SOURCE_FILES
    src/main.cpp
)  

# Eigen Matrix Library
find_package(Eigen3 REQUIRED NO_MODULE)

# Compile executable
add_executable(EKF ${SOURCE_FILES})

