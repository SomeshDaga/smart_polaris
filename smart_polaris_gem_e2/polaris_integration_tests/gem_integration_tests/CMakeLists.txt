cmake_minimum_required(VERSION 3.0.2)
project(gem_integration_tests)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/MockStateSignals.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

catkin_install_python(PROGRAMS
  scripts/status_overlay_node.py
  scripts/mock_state_signals_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Integration tests (rostest): launch full stack + run scenario tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/integration_test_scenarios.test)
endif()

## Install test scripts and launch for rostest
catkin_install_python(PROGRAMS test/integration_test_scenarios.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES test/integration_test_scenarios.test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
