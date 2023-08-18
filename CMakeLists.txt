cmake_minimum_required(VERSION 3.0.2)
project(tf_viz)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf
  tf2_ros
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tf_viz
  CATKIN_DEPENDS geometry_msgs roscpp rospy tf tf2_ros visualization_msgs
#  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(tf_viz src/tf_viz.cpp)
target_link_libraries(tf_viz ${catkin_LIBRARIES})