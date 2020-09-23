cmake_minimum_required(VERSION 3.0.0)

project(robot_wrapper)

add_definitions(-DLINUX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Moc generated files are located in the current dir so we need to tell CMake to look for them.
set(CMAKE_INCLUDE_CURRENT_DIR ON)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
  robo_lib
  thread_lib
  lwr4p
  ur_robot
  ati_sensor
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
  DEPENDS
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

set(SRC_LIST
  src/robot.cpp
  src/lwr4p_robot.cpp
  src/ur_robot.cpp
  src/utils/math_utils.cpp
)

add_library(${PROJECT_NAME}
  ${SRC_LIST}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  armadillo
  robo_lib
  thread_lib
  lwr4p
  ur_robot
  ati_sensor
  pthread
)

# =====================================================
