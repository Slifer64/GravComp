cmake_minimum_required(VERSION 2.8.3)
project(ur_robot)

add_definitions(-DLINUX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules like armadillo
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../install/deps/ ${CMAKE_MODULE_PATH} )
# set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH} )

find_package(orocos_kdl)
find_package(catkin REQUIRED COMPONENTS
  orocos_kdl
  kdl_parser
  roscpp
  roslib
  geometry_msgs
  sensor_msgs
  # ur_kinematics
  ur_modern_driver
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp ur_modern_driver
  DEPENDS orocos_kdl
)

set(SRC_LIST
  src/robot_urdf.cpp
  src/robot_arm.cpp
  src/robot.cpp
  src/sim_robot.cpp
  src/utils.cpp
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${ARMADILLO_INCLUDE_DIR}
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

# *************************************************
# *************************************************

add_library(${PROJECT_NAME}
  ${SRC_LIST}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  pthread
  orocos-kdl
  armadillo
  ${catkin_LIBRARIES}
  ur_modern_driver
)


# *************************************************
# *************************************************
