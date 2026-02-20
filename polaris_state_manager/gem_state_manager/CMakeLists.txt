cmake_minimum_required(VERSION 3.0.2)
project(gem_state_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gem_state_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp gem_state_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_manager_node
  nodes/state_manager_node.cpp
  src/state_manager.cpp
)
add_dependencies(state_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_manager_node
  ${catkin_LIBRARIES}
)