cmake_minimum_required(VERSION 3.0.2)
project(sample_rviz_arrow_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  pluginlib
  rviz
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plugin_lecture
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

#Qt5 configulation
set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/rviz/sample_initialpose_tool.cpp
) 

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
