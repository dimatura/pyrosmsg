cmake_minimum_required(VERSION 2.8.3)
project(pyrosmsg)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  nav_msgs
  pybind11_catkin
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(NUMPY REQUIRED)
find_package(pybind11 REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    cmake_modules
    geometry_msgs
    nav_msgs
    pybind11_catkin
    roscpp
    rospy
    sensor_msgs
    std_msgs
    visualization_msgs
    #DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    #${PYBIND11_INCLUDE_DIRS}
)

message("pybind11_include_dir " ${PYBIND11_INCLUDE_DIRS})
pybind_add_module(libpyrosmsg
    src_wrap/module.cpp
)
