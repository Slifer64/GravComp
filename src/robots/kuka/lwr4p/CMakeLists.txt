cmake_minimum_required(VERSION 2.8.3)

project(lwr4p)

add_definitions(-DLINUX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules like armadillo
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../install/deps/ ${CMAKE_MODULE_PATH} )
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH} )

find_package(orocos_kdl)
find_package(FastResearchInterfaceLibrary REQUIRED)
find_package(catkin REQUIRED
  orocos_kdl
  kdl_parser
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include ${FastResearchInterfaceLibrary_INCLUDE_DIR}
  CATKIN_DEPENDS roscpp
  DEPENDS orocos_kdl
)

SET(SRC_LIST
  src/utils.cpp
  src/robot_arm.cpp
  src/robot.cpp
  src/sim_robot.cpp
  src/robot_urdf.cpp
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
  ${FastResearchInterfaceLibrary_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  ${SRC_LIST}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  pthread
  orocos-kdl
  armadillo
  ${catkin_LIBRARIES}
  ${FastResearchInterfaceLibrary_LIBRARIES}
)
