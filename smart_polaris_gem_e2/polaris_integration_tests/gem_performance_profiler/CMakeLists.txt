cmake_minimum_required(VERSION 3.0.2)
project(gem_performance_profiler)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/fake_get_model_state_node.py
  scripts/run_error_timing_profiler.py
  scripts/analyze_rosbags.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
