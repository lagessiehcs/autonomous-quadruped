cmake_minimum_required(VERSION VERSION 3.0.2)
project(controller)

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
  geometry_msgs 
  mav_msgs
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  mav_msgs 
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(controller src/controller.cpp)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(controller ${catkin_LIBRARIES})
