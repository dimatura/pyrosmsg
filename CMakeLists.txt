cmake_minimum_required(VERSION 2.8.3)
project(pymsg)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

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
find_package(Eigen3 3.3.1 REQUIRED)

set(PY_PROJECT_NAME ${PROJECT_NAME})
set(PY_PACKAGE_DIR python/${PROJECT_NAME})

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    cmake_modules
    geometry_msgs
    nav_msgs
    pybind11_module
    roscpp
    rospy
    sensor_msgs
    std_msgs
    visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
    include
    ${Eigen3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
    src_wrap/module.cpp
)

target_link_libraries(${PY_PROJECT_NAME}
    ${catkin_LIBRARIES}
)
