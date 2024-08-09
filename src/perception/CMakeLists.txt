cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS 
  rospy 
  mav_msgs
)

catkin_package(CATKIN_DEPENDS 
  rospy 
  mav_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
