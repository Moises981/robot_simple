cmake_minimum_required(VERSION 3.0.2)
project(robot_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  pluginlib
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_controller
  CATKIN_DEPENDS controller_interface controller_manager hardware_interface pluginlib roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/robot_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
