cmake_minimum_required(VERSION VERSION 3.0.2)
project(teleop_actuators_keyboard)

find_package(catkin REQUIRED COMPONENTS 
  rospy 
  mav_msgs
)

catkin_package(CATKIN_DEPENDS 
  rospy 
  mav_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/teleop_actuators_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
