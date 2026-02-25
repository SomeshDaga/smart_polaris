cmake_minimum_required(VERSION 3.0.2)
project(gem_task_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gem_controller_action
  gem_state_msgs
)

catkin_package(
  CATKIN_DEPENDS gem_controller_action gem_state_msgs
)

catkin_install_python(
  PROGRAMS scripts/task_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
