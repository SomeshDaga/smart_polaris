#!/usr/bin/env python3
"""
Performance profiler: run scenarios (running->error or error->running) while
recording rosbags for timing analysis.

Scenarios:
  run_to_error:  IDLE -> start bag -> startup delay -> RUNNING -> inject error ->
                 wait ERROR -> sleep -> stop bag.
  error_to_run:  IDLE -> start bag -> startup delay -> RUNNING -> inject error ->
                 wait ERROR -> clear error -> wait RUNNING (recovery) -> sleep -> stop bag.

Requires the profile pipeline to be running (roslaunch gem_performance_profiler profile_pipeline.launch).
"""
from __future__ import print_function

import argparse
import os
import sys
import time

import rospy

from gem_performance_profiler.performance_scenarios import (
    SIGNALS,
    SCENARIOS,
    SCENARIO_RUN_TO_ERROR,
    SCENARIO_ERROR_TO_RUN,
    run_scenario_with_rosbag,
)

RUNS_PER_SIGNAL = 100


def resolve_waypoints_file():
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
    return waypoints_file


def main():
    parser = argparse.ArgumentParser(
        description="Run error-timing profiling: record rosbags for run_to_error or error_to_run scenarios.",
    )
    parser.add_argument(
        "--scenario",
        type=str,
        choices=list(SCENARIOS.keys()),
        default=SCENARIO_RUN_TO_ERROR,
        help="Scenario to run: run_to_error (running->error) or error_to_run (recovery). Default: run_to_error.",
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
        test_dir = os.path.abspath(
            "profile_{}_{}".format(args.scenario, time.strftime("%Y%m%d_%H%M%S"))
        )
    print("Test directory: {}".format(test_dir), file=sys.stderr)
    print("Scenario: {}".format(args.scenario), file=sys.stderr)
    sys.stderr.flush()

    rospy.init_node("run_error_timing_profiler", anonymous=False)

    waypoints_file = resolve_waypoints_file()
    if not os.path.isfile(waypoints_file):
        rospy.logerr("Waypoints file not found: {}".format(waypoints_file))
        sys.exit(1)

    rospy.wait_for_service("/serve_waypoints", timeout=30)
    rospy.wait_for_service("/cancel_tasks", timeout=10)
    from gem_controller_action.srv import ServeWaypoints
    from std_srvs.srv import Empty, EmptyRequest
    serve_waypoints_proxy = rospy.ServiceProxy("/serve_waypoints", ServeWaypoints)
    cancel_tasks_proxy = rospy.ServiceProxy("/cancel_tasks", Empty)

    scenario = SCENARIOS[args.scenario]

    for signal_name in SIGNALS:
        rospy.loginfo("Signal: {} ({} runs)".format(signal_name, args.runs))
        ok_count = 0
        for run_id in range(1, args.runs + 1):
            if run_scenario_with_rosbag(
                scenario,
                test_dir,
                signal_name,
                run_id,
                waypoints_file,
                serve_waypoints_proxy,
                cancel_tasks_proxy,
            ):
                ok_count += 1
            print("\r  run {}/{} (ok: {})   ".format(run_id, args.runs, ok_count), end="", file=sys.stderr)
            sys.stderr.flush()
        print(file=sys.stderr)
        rospy.loginfo("Signal {}: {} / {} runs succeeded".format(signal_name, ok_count, args.runs))

    print("Test directory: {}".format(test_dir), file=sys.stderr)
    rospy.loginfo("Profiling complete. Test directory: {}".format(test_dir))


if __name__ == "__main__":
    main()
