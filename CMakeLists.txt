cmake_minimum_required(VERSION 2.8)

project(testmod)
set(PYBIND11_CPP_STANDARD -std=c++11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -O2")
add_subdirectory(pybind11)

pybind11_add_module(mymod mymod.cpp control_joint.cpp control_turnover.cpp control_sliding.cpp control_pegin.cpp robot.cpp kyomath.cpp nn_interface.cpp pancake.cpp)
