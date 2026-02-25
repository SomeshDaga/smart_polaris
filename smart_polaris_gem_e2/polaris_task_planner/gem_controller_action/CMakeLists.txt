cmake_minimum_required(VERSION 3.0.2)
project(gem_controller_action)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    message_generation
    ackermann_msgs
    gazebo_msgs
    geometry_msgs)

add_service_files(
  DIRECTORY srv
  FILES
    ServeWaypoints.srv
)

add_action_files(
  DIRECTORY action
  FILES
    NavigateWaypoints.action
)

catkin_python_setup()

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib actionlib_msgs ackermann_msgs gazebo_msgs geometry_msgs)