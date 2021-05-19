cmake_minimum_required(VERSION 2.8.3)
project(airsim_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  quadrotor_msgs
  tf
  nodelet
  cmake_utils
  airsim_ros_pkgs
)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES irobot_msgs
#  CATKIN_DEPENDS geometry_msgs nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
#find_package(Armadillo REQUIRED)
#include_directories(${ARMADILLO_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(irobot_msgs
#   src/${PROJECT_NAME}/irobot_msgs.cpp
# )

## Declare a cpp executable
#add_executable(odom_visualization src/odom_visualization.cpp)
add_library(AirsimController src/AirsimController.cpp)
add_library(airsim_controller_nodelet src/airsim_controller_nodelet.cpp)

target_link_libraries(airsim_controller_nodelet
    ${catkin_LIBRARIES}
        AirsimController
)