#!/usr/bin/env python3
"""
Performance profiler: for each error-inducing signal, run 100 trials that:
  1. Start system in IDLE
  2. Start rosbag record (from IDLE)
  3. Put task planner in RUNNING (serve_waypoints)
  4. Inject error for the signal via mock_state_signals (dynamic_reconfigure)
  5. Stop rosbag N seconds after injecting the error (5s default; 25s for network_strength_low_signal, 15s for network_strength_not_connected)

Requires the profile pipeline to be running (roslaunch gem_performance_profiler profile_pipeline.launch).
"""
from __future__ import print_function

import argparse
import os
import signal
import subprocess
import sys
import time

import rospy
from dynamic_reconfigure.client import Client as DynReconfClient
from gem_state_msgs.msg import SystemStateStamped, TaskPlannerStatus, SignalStrengthStamped
from gem_controller_action.srv import ServeWaypoints, ServeWaypointsRequest
from std_srvs.srv import Empty, EmptyRequest

# Signals we profile (subdir names and error-injection config)
SIGNALS = [
    "estop",
    "battery_level",
    "temperature",
    "gps_accuracy",
    "network_strength_low_signal",
    "network_strength_not_connected",
]
RUNS_PER_SIGNAL = 100

ROSBAG_STARTUP_DELAY = 2.0  # Give rosbag some time to start recording and settle before generating events
ROSBAG_DURATION_SEC_DEFAULT = 5.0

WAIT_IDLE_TIMEOUT = 5.0
WAIT_RUNNING_TIMEOUT = 5.0  # Timeout to wait for system to go into running mode after giving a task

# We have to wait different amounts of time for system to go into error state
# after inducing the error signal (depending on the type of error)
DEFAULT_WAIT_ERROR_TIMEOUT = 5.0
WAIT_ERROR_TIMEOUT = {
    "gps_accuracy": 15.0 + DEFAULT_WAIT_ERROR_TIMEOUT,
    "network_strength_low_signal": 20.0 + DEFAULT_WAIT_ERROR_TIMEOUT,
    "network_strength_not_connected": 10.0 + DEFAULT_WAIT_ERROR_TIMEOUT,
}
ROSBAG_DURATION_AFTER_ERROR = 3.0

POLL_PERIOD = 0.2
MOCK_RECONF_NODE = "/mock_state_signals_node"


def _good_config():
    return {
        "estop_enabled": False,
        "battery_level": 100,
        "gps_accuracy": 100.0,
        "signal_strength": SignalStrengthStamped.CONNECTED,
        "temperature": 30.0,
    }


def _error_config_for_signal(signal_name):
    """Return dynamic_reconfigure dict that induces error for the given signal."""
    base = _good_config()
    if signal_name == "battery_level":
        base["battery_level"] = 49
    elif signal_name == "estop":
        base["estop_enabled"] = True
    elif signal_name == "temperature":
        base["temperature"] = 60.0
    elif signal_name == "gps_accuracy":
        base["gps_accuracy"] = 201.0
    elif signal_name == "network_strength_low_signal":
        base["signal_strength"] = SignalStrengthStamped.LOW_SIGNAL
    elif signal_name == "network_strength_not_connected":
        base["signal_strength"] = SignalStrengthStamped.NOT_CONNECTED
    else:
        raise ValueError("Unknown signal: {}".format(signal_name))
    return base


def set_mock_config(config_dict):
    client = DynReconfClient(MOCK_RECONF_NODE, timeout=5.0)
    current = client.get_configuration()
    for k, v in config_dict.items():
        current[k] = v
    client.update_configuration(current)


def get_system_state():
    try:
        msg = rospy.wait_for_message("/system_state", SystemStateStamped, timeout=2.0)
        return msg.state
    except rospy.ROSException:
        return None


def get_task_status():
    try:
        msg = rospy.wait_for_message("/task_planner_status", TaskPlannerStatus, timeout=2.0)
        return msg.status
    except rospy.ROSException:
        return None


def poll_until(condition, timeout_sec, desc="condition"):
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    while rospy.Time.now() < deadline:
        if condition():
            return True
        rospy.sleep(POLL_PERIOD)
    return False


def start_rosbag_record(bag_path):
    """Start rosbag record in background; returns Popen process. Caller must stop it (e.g. after 5s past error)."""
    cmd = ["rosbag", "record", "-a", "-O", bag_path]
    return subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )


def stop_rosbag_record(process, timeout_sec=10):
    """Stop rosbag cleanly (SIGINT) and wait for it to finish writing the bag."""
    if process is None or process.poll() is not None:
        return
    try:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
    except Exception:
        try:
            process.kill()
        except Exception:
            pass


def one_run(test_dir, signal_name, run_id, waypoints_file, serve_waypoints_proxy, cancel_tasks_proxy):
    subdir = os.path.join(test_dir, signal_name)
    os.makedirs(subdir, exist_ok=True)
    bag_name = "run_{:03d}.bag".format(run_id)
    bag_path = os.path.join(subdir, bag_name)

    # Reset to good state and cancel any task
    set_mock_config(_good_config())
    try:
        cancel_tasks_proxy(EmptyRequest())
    except rospy.ServiceException:
        pass
    rospy.sleep(0.5)

    # Wait IDLE (so we start recording from IDLE)
    if not poll_until(
        lambda: get_system_state() == SystemStateStamped.STATE_IDLE,
        WAIT_IDLE_TIMEOUT,
        "IDLE",
    ):
        rospy.logwarn("Run {} signal {}: did not reach IDLE in time".format(run_id, signal_name))
        return False

    # Start rosbag record now (from IDLE); stop 5s after error injection
    rosbag_proc = start_rosbag_record(bag_path)
    try:
        # Give rosbag some time to start recording and settle
        # Otherwise, some topics may be subscribed before others and
        # we may missing important starting events 
        rospy.sleep(ROSBAG_STARTUP_DELAY)
        # Start task
        try:
            resp = serve_waypoints_proxy(ServeWaypointsRequest(waypoints_file=waypoints_file))
            if not resp.success:
                rospy.logwarn("Run {} signal {}: serve_waypoints failed".format(run_id, signal_name))
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("Run {} signal {}: serve_waypoints error: {}".format(run_id, signal_name, e))
            return False

        # Wait RUNNING (system + task planner)
        if not poll_until(
            lambda: get_system_state() == SystemStateStamped.STATE_RUNNING
            and get_task_status() == TaskPlannerStatus.STATUS_RUNNING_TASK,
            WAIT_RUNNING_TIMEOUT,
            "RUNNING",
        ):
            rospy.logwarn("Run {} signal {}: did not reach RUNNING in time".format(run_id, signal_name))
            return False

        # Inject error
        set_mock_config(_error_config_for_signal(signal_name))

        # Wait ERROR (optional for bag; we still want 5s past injection)
        poll_until(
            lambda: get_system_state() == SystemStateStamped.STATE_ERROR,
            WAIT_ERROR_TIMEOUT.get(signal_name, DEFAULT_WAIT_ERROR_TIMEOUT),
            "ERROR",
        )

        # Record until N seconds past error injection to ensure we capture any
        # events that are caused by the error within a reasonable time window.
        rospy.sleep(ROSBAG_DURATION_AFTER_ERROR)
        return True
    finally:
        stop_rosbag_record(rosbag_proc)


def main():
    parser = argparse.ArgumentParser(
        description="Run error-timing profiling: 100 runs per signal, rosbag 5s past error."
    )
    parser.add_argument(
        "--test-dir",
        type=str,
        default=None,
        help="Root directory for run subdirs (one per signal). If omitted, a timestamped dir is created and printed.",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=RUNS_PER_SIGNAL,
        help="Number of runs per signal (default %d)." % RUNS_PER_SIGNAL,
    )
    args = parser.parse_args()

    if args.test_dir:
        test_dir = os.path.abspath(args.test_dir)
    else:
        test_dir = os.path.abspath("profile_{}".format(time.strftime("%Y%m%d_%H%M%S")))
    print("Test directory: {}".format(test_dir), file=sys.stderr)
    sys.stderr.flush()

    rospy.init_node("run_error_timing_profiler", anonymous=False)

    waypoints_file = rospy.get_param("waypoints_file", "")
    if not waypoints_file:
        try:
            import rospkg
            rp = rospkg.RosPack()
            pkg_path = rp.get_path("gem_integration_tests")
            waypoints_file = os.path.join(pkg_path, "config", "wps.csv")
        except (rospkg.ResourceNotFound, ImportError):
            waypoints_file = os.path.join(
                os.path.dirname(__file__), "..", "..", "gem_integration_tests", "config", "wps.csv"
            )
    if not os.path.isabs(waypoints_file):
        waypoints_file = os.path.abspath(waypoints_file)
    if not os.path.isfile(waypoints_file):
        rospy.logerr("Waypoints file not found: {}".format(waypoints_file))
        sys.exit(1)

    rospy.wait_for_service("/serve_waypoints", timeout=30)
    rospy.wait_for_service("/cancel_tasks", timeout=10)
    serve_waypoints_proxy = rospy.ServiceProxy("/serve_waypoints", ServeWaypoints)
    cancel_tasks_proxy = rospy.ServiceProxy("/cancel_tasks", Empty)

    for signal_name in SIGNALS:
        rospy.loginfo("Signal: {} ({} runs)".format(signal_name, args.runs))
        ok_count = 0
        for run_id in range(1, args.runs + 1):
            if one_run(
                test_dir,
                signal_name,
                run_id,
                waypoints_file,
                serve_waypoints_proxy,
                cancel_tasks_proxy,
            ):
                ok_count += 1
            # Overwrite same progress line after each run
            print("\r  run {}/{} (ok: {})   ".format(run_id, args.runs, ok_count), end="", file=sys.stderr)
            sys.stderr.flush()
        print(file=sys.stderr)  # newline after signal done
        rospy.loginfo("Signal {}: {} / {} runs succeeded".format(signal_name, ok_count, args.runs))

    print("Test directory: {}".format(test_dir), file=sys.stderr)
    rospy.loginfo("Profiling complete. Test directory: {}".format(test_dir))


if __name__ == "__main__":
    main()
