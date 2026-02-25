cmake_minimum_required(VERSION 2.8.3)
project(gem_pure_pursuit_sim_v2)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gem_controller_action
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  ackermann_msgs
  visualization_msgs
)

catkin_package(
)

catkin_install_python(
  PROGRAMS scripts/pure_pursuit_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


