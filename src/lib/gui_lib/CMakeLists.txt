cmake_minimum_required(VERSION 3.0.0)

project(gui_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_STANDARD 11) # Tell CMake to compile with C++11
set(CMAKE_AUTOMOC ON) # Tell CMake to run moc when needed.
set(CMAKE_AUTOUIC ON) # Tell CMake to run uic when needed.
set(CMAKE_AUTORCC ON) # Tell CMake to run rcc when needed

# Moc generated files are located in the current dir so we need to tell CMake to look for them.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets Core Gui PrintSupport REQUIRED)
# Add Qt5 definitions and includes to build libraries.
# Widgets add Widgets Core and Gui
add_definitions(${Qt5Widgets_DEFINITIONS})
include_directories(${Qt5Widgets_INCLUDES})
SET(Qt_LIBRARIES
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::PrintSupport
)


find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/view_wrench_dialog.h
  src/view_wrench_dialog.cpp

  include/${PROJECT_NAME}/view_jpos_dialog.h
  src/view_jpos_dialog.cpp

  include/${PROJECT_NAME}/view_pose_dialog.h
  src/view_pose_dialog.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Qt_LIBRARIES}
  armadillo
)
