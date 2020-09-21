cmake_minimum_required(VERSION 2.8.3)

project(thread_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
  roscpp
  roslib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

SET(SRC_LIST
  src/thread_utils.cpp
)

include_directories(
  include
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
  ${catkin_LIBRARIES}
)
