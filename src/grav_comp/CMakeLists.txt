cmake_minimum_required(VERSION 3.0.0)

project(grav_comp)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  tf
  io_lib
  math_lib
  robo_lib
  rviz_lib
  gui_lib
  robot_wrapper
  thread_lib
)
 
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs tf
  DEPENDS
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

set(SRC_LIST
  include/${PROJECT_NAME}/gui/mainwindow.cpp
  include/${PROJECT_NAME}/gui/utils.cpp
  include/${PROJECT_NAME}/gui/set_poses_dialog.cpp

  src/source_list/grav_comp.cpp
  src/main.cpp
)

add_executable(${PROJECT_NAME}
  ${SRC_LIST}
  resources/resource.qrc
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  armadillo
  io_lib
  math_lib
  robo_lib
  rviz_lib
  gui_lib
  robot_wrapper
  thread_lib
  ${Qt_LIBRARIES}
  pthread
)
