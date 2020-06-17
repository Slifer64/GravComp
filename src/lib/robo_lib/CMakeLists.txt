cmake_minimum_required(VERSION 2.8.3)

project(robo_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(orocos_kdl)
find_package(catkin REQUIRED
  roscpp
  roslib
  orocos_kdl
  kdl_parser
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS orocos_kdl
)

SET(SRC_LIST
  src/kinematic_chain.cpp
  src/ppc_joint_limit_avoid.cpp
  src/singular_value_filter.cpp
  src/tool_estimator.cpp
  src/joint_state_publisher
  src/xml_parser.cpp
)

include_directories(
  include
  usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SRC_LIST}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  armadillo
  orocos-kdl
  ${catkin_LIBRARIES}
)
