cmake_minimum_required(VERSION 2.8.3)
project(pymsg)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  nav_msgs
  pybind11_module
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(NUMPY REQUIRED)
find_package(Eigen3 REQUIRED)

set(PY_PROJECT_NAME ${PROJECT_NAME})
set(PY_PACKAGE_DIR python/${PROJECT_NAME})

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs 
    visualization_msgs
    cmake_modules
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
    src_wrap/module.cpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(${PY_PROJECT_NAME}
  ${catkin_LIBRARIES}
)

