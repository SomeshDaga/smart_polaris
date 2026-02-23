# gem_integration_tests

Integration tests for the GEM stack: **state_manager**, **task_planner**, and **pure_pursuit_sim** with Gazebo and RViz.

## Components

- **state_manager_node** (`gem_state_manager`): Publishes `/system_state` from battery, estop, gps, signal, task status.
- **task_planner_node** (`gem_task_planner`): `serve_waypoints` service, `/task_planner_status`, and `navigate_waypoints` action client.
- **pure_pursuit_sim** (`gem_pure_pursuit_sim_v2`): `navigate_waypoints` action server; publishes `/gem/ackermann_cmd`.

## Config and RViz

- **config/wps.csv**: Waypoints used for navigation (loaded via `serve_waypoints` with this path or via param `waypoints_file`).
- **rviz/gem_integration_tests.rviz**: RViz config based on the default Gazebo RViz, with an overlay showing:
  - `navigate_waypoints` action client status
  - `/system_state` (state)
  - `/task_planner_status` (status)
  - `/battery_level`, `/gps_accuracy`, `/signal_strength`, `/estop`, `/gem/ackermann_cmd` (speed)

The overlay is published by `status_overlay_node.py` on `/gem_integration_tests/status_overlay` (jsk_rviz_plugins/OverlayText).

## Running the simulation (visible for video)

From inside the container (`docker exec -it steerai_dev bash`), after sourcing the workspace:

```bash
source /opt/ros/noetic/setup.bash && source /home/ros/steer_polaris_ws/devel/setup.bash
roslaunch gem_integration_tests integration_test.launch
```

This starts Gazebo (GUI), RViz (with integration status overlay), state_manager, mock_state_signals, pure_pursuit_sim, task_planner, and status_overlay_node. You can send a waypoint goal via:

```bash
rosservice call /serve_waypoints "waypoints_file: '$(rospack find gem_integration_tests)/config/wps.csv'"
```

Mock signals are controlled by ROS params (e.g. set estop to trigger ERROR):

```bash
rosparam set /mock_state_signals_node/estop_enabled true   # ERROR
rosparam set /mock_state_signals_node/estop_enabled false  # clear
```

## Running the integration tests (rostest)

From inside the container, with the workspace built and sourced:

```bash
source /opt/ros/noetic/setup.bash && source /home/ros/steer_polaris_ws/devel/setup.bash
catkin test gem_integration_tests
```

Or run the rostest launch directly (headless: no Gazebo GUI, no RViz):

```bash
rostest gem_integration_tests integration_test_scenarios.test
```

## Test scenarios

1. **Waypoint goal while ERROR**: Estop enabled → system in ERROR. Send waypoint goal. Assert no active/pending goal to `navigate_waypoints` and `/task_planner_status` is `STATUS_PAUSED_TASK`.
2. **Waypoint goal while IDLE**: System in IDLE. Send waypoint goal. Assert navigation starts and `/task_planner_status` becomes `STATUS_RUNNING_TASK`.
3. **Signal bad during navigation**: While navigating, set estop. Assert system goes to ERROR, task to `STATUS_PAUSED_TASK`, goal preempted, and `/gem/ackermann_cmd` speed 0.
4. **Signal resolves**: Clear estop. Assert system goes to RUNNING, `STATUS_RUNNING_TASK`, and `/gem/ackermann_cmd` speed > 1.0.

## Dependencies

- `gem_gazebo`, `gem_state_manager`, `gem_task_planner`, `gem_pure_pursuit_sim_v2`, `gem_controller_action`, `gem_state_msgs`, `jsk_rviz_plugins`, `actionlib`, `actionlib_msgs`, `ackermann_msgs`, `rviz`, `rostest`
