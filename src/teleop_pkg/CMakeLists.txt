cmake_minimum_required(VERSION 2.8.3)
project(teleop_pkg)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  mav_msgs
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  mav_msgs
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(teleop_subscriber_node src/teleop_subscriber_node.cpp)
add_dependencies(teleop_subscriber_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_subscriber_node ${catkin_LIBRARIES})
