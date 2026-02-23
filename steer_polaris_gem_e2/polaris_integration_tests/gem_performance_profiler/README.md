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

With a **specific test directory**:

```bash
docker exec -it steerai_dev bash
source /opt/ros/noetic/setup.bash && source /home/ros/steer_polaris_ws/devel/setup.bash
rosrun gem_performance_profiler run_error_timing_profiler.py --test-dir /tmp/my_profile
```

With a **dynamically created** test directory (timestamped; path is printed):

```bash
rosrun gem_performance_profiler run_error_timing_profiler.py
```

Optional: `--runs N` to use N runs per signal (default 100).

### Output layout

- **Test directory** (e.g. `/tmp/my_profile` or `profile_YYYYMMDD_HHMMSS`):
  - `battery_level/run_001.bag` … `run_100.bag`
  - `temperature/run_001.bag` … `run_100.bag`
  - `gps_accuracy/run_001.bag` … `run_100.bag`
  - `network_strength_low_signal/run_001.bag` … `run_100.bag`
  - `network_strength_not_connected/run_001.bag` … `run_100.bag`

Each run: system starts IDLE → task planner put in RUNNING → error injected for that signal → rosbag recorded until N seconds past the error (see script for per-signal durations).

### 3. Analyze rosbags (after profiling)

From a test directory containing signal subdirs and `run_*.bag` files:

```bash
rosrun gem_performance_profiler analyze_rosbags.py /path/to/test_dir [--output-dir /path/to/output]
```

**Requires:** `pandas`, `matplotlib`, `numpy` (e.g. `pip install pandas matplotlib numpy`).

**Outputs** (in `test_dir` or `--output-dir`):
- **event_times_and_latencies.csv** — one row per bag: columns for time of error injection, time `/system_state` goes to ERROR, time `/task_planner_status` goes to PAUSED, time `/navigate_waypoints/status` shows PREEMPTED, time `/gem/ackermann_cmd` shows zero speed, plus latency columns (each event time minus error injection time).
- **latency_statistics.txt** — for each event column: number of valid instances (success rate %), mean reaction time (s), std (s), 95th percentile (s) w.r.t. error-inducing message timestamp.
- **histogram_*.png** — histogram of latency for each event (overall and per-signal).

Times use message `header.stamp` when available, otherwise the bag-assigned time.

## Error-inducing signals

| Signal | Mock config change | State manager error (after persistence where applicable) |
|--------|---------------------|------------------------------------------------------------|
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
