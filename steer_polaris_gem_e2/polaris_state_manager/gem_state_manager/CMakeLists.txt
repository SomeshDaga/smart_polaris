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

# Rostest for state manager scenarios test
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(state_manager_scenarios test/state_manager_scenarios.test test/state_manager_scenarios.cpp)
  target_link_libraries(state_manager_scenarios ${catkin_LIBRARIES})
  add_dependencies(state_manager_scenarios ${catkin_EXPORTED_TARGETS})
endif()