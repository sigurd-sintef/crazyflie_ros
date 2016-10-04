cmake_minimum_required(VERSION 2.8.3)
project(easyfly)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  crazyflie_cpp
)

# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS program_options REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES easyfly
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS
  crazyflie_cpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fly
  src/fly.cpp
)

target_link_libraries(fly
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


add_executable(joysticks2
  src/joysticks2.cpp
)

target_link_libraries(joysticks2
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
