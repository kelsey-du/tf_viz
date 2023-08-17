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



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_broad src/tf_broad.cpp)
target_link_libraries(tf_broad ${catkin_LIBRARIES})