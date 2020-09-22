cmake_minimum_required(VERSION 2.8.3)
project(pyrosmsg)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  trajectory_msgs
  nav_msgs
  pybind11_catkin
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  moveit_msgs
)

find_package(NUMPY REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    cmake_modules
    geometry_msgs
    trajectory_msgs
    nav_msgs
    pybind11_catkin
    moveit_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    visualization_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

pybind_add_module(libpyrosmsg
    src_wrap/module.cpp
)
