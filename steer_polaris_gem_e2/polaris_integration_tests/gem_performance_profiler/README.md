# gem_performance_profiler

Profiles timing performance of the pipeline (state_manager_node, task_planner_node, mock_state_signals_node, pure_pursuit_sim) for error-injection scenarios **without Gazebo**. A fake `/gazebo/get_model_state` service is provided so the controller action interface can run.

## Usage

### 1. Start the pipeline (no Gazebo)

```bash
source /opt/ros/noetic/setup.bash && source /home/ros/steer_polaris_ws/devel/setup.bash
roslaunch gem_performance_profiler profile_pipeline.launch
```

Optionally override the waypoints file used by the profiler script:

```bash
roslaunch gem_performance_profiler profile_pipeline.launch waypoints_file:=$(find gem_integration_tests)/config/wps.csv
```

### 2. Run the error-timing profiler (in another terminal)

**Scenarios:**
- **run_to_error** (default): System in RUNNING → inject error → wait ERROR → record for `ROSBAG_DURATION_AFTER_ERROR` then stop. Evaluates reaction to fault.
- **error_to_run**: System in RUNNING → inject error → wait ERROR → clear error → wait RUNNING again (system + task + ackermann speed > 0) → record for `ROSBAG_DURATION_AFTER_ERROR` then stop. Evaluates recovery.

Both use the same `ROSBAG_STARTUP_DELAY` and `ROSBAG_DURATION_AFTER_ERROR` (see `profiler_scenarios.py`).

With a **specific test directory**:

```bash
docker exec -it steerai_dev bash
source /opt/ros/noetic/setup.bash && source /home/ros/steer_polaris_ws/devel/setup.bash
rosrun gem_performance_profiler run_error_timing_profiler.py --scenario run_to_error --test-dir /tmp/my_profile
rosrun gem_performance_profiler run_error_timing_profiler.py --scenario error_to_run --test-dir /tmp/my_recovery
```

With a **dynamically created** test directory (timestamped; path is printed):

```bash
rosrun gem_performance_profiler run_error_timing_profiler.py --scenario run_to_error
```

Optional: `--runs N` to use N runs per signal (default 100).

### Output layout

- **Test directory** (e.g. `/tmp/my_profile` or `profile_run_to_error_YYYYMMDD_HHMMSS`):
  - `estop/run_001.bag` … `run_100.bag`
  - `battery_level/run_001.bag` … `run_100.bag`
  - … (one subdir per signal)

Each run: IDLE → start bag → startup delay → RUNNING → (inject error → ERROR) and for error_to_run then (clear error → RUNNING again) → sleep → stop bag.

### 3. Analyze rosbags (after profiling)

From a test directory containing signal subdirs and `run_*.bag` files. **Match `--scenario` to how the bags were recorded.**

```bash
rosrun gem_performance_profiler analyze_rosbags.py /path/to/test_dir --scenario run_to_error [--output-dir /path/to/output]
rosrun gem_performance_profiler analyze_rosbags.py /path/to/recovery_dir --scenario error_to_run [--output-dir /path/to/output]
```

**Requires:** `pandas`, `matplotlib`, `numpy` (e.g. `pip install pandas matplotlib numpy`).

**Outputs** (in `test_dir` or `--output-dir`):

- **run_to_error:** `event_times_and_latencies_run_to_error.csv` — columns: time_error_injection, time_system_state_error, time_task_planner_paused, time_nav_preempted, time_ackermann_zero, plus latency_* w.r.t. time_error_injection. `latency_statistics_run_to_error.txt`, `run_to_error_histogram_*.png`.
- **error_to_run:** `event_times_and_latencies_error_to_run.csv` — columns: time_error_cleared, time_system_state_running, time_task_planner_running, time_nav_active, time_ackermann_positive, plus latency_* w.r.t. time_error_cleared. `latency_statistics_error_to_run.txt`, `error_to_run_histogram_*.png`.

Times use message `header.stamp` when available, otherwise the bag-assigned time.

## Error-inducing signals

| Signal | Mock config change | State manager error (after persistence where applicable) |
|--------|---------------------|------------------------------------------------------------|
| estop | estop_enabled = true | ERROR_ESTOP (immediate) |
| battery_level | battery_level = 49% | ERROR_BATTERY_LOW (immediate) |
| temperature | temperature = 60°C | ERROR_TEMPERATURE_HIGH (immediate) |
| gps_accuracy | gps_accuracy = 201 mm | ERROR_GPS_LOST (bad > 15 s) |
| network_strength_low_signal | signal_strength = LOW_SIGNAL | ERROR_SIGNAL_LOW (low > 20 s) |
| network_strength_not_connected | signal_strength = NOT_CONNECTED | ERROR_SIGNAL_LOST (lost > 10 s) |

## Nodes in the pipeline

- **fake_get_model_state**: Advertises `/gazebo/get_model_state` (required by `controller_action_interface` when Gazebo is not running).
- **state_manager**: Subscribes to battery, estop, gps_accuracy, signal_strength, temperature, task_planner_status; publishes `/system_state`.
- **mock_state_signals_node**: Publishes mock state topics; parameters set via dynamic_reconfigure.
- **pure_pursuit_sim**: Action server `navigate_waypoints`, uses GetModelState and publishes `/gem/ackermann_cmd`.
- **task_planner**: Service `serve_waypoints`, subscribes to `/system_state`, sends goals to the controller.
