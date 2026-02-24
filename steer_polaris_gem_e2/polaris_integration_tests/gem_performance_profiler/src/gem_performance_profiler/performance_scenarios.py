"""
Shared scenario logic for the error-timing profiler.

Scenarios define what runs while the rosbag is recording. Each scenario
implements setup() (before rosbag) and run() (while recording).
"""
from __future__ import print_function

import os
import signal
import subprocess

import rospy
from dynamic_reconfigure.client import Client as DynReconfClient
from ackermann_msgs.msg import AckermannDrive
from gem_state_msgs.msg import SystemStateStamped, TaskPlannerStatus, SignalStrengthStamped
from gem_controller_action.srv import ServeWaypoints, ServeWaypointsRequest
from std_srvs.srv import Empty, EmptyRequest

# -----------------------------------------------------------------------------
# Constants (shared with analyzer via import)
# -----------------------------------------------------------------------------
SIGNALS = [
    "estop",
    "battery_level",
    "temperature",
    "gps_accuracy",
    "network_strength_low_signal",
    "network_strength_not_connected",
]

ROSBAG_STARTUP_DELAY = 2.0
ROSBAG_DURATION_AFTER_ERROR = 3.0

WAIT_IDLE_TIMEOUT = 5.0
WAIT_RUNNING_TIMEOUT = 5.0
DEFAULT_WAIT_ERROR_TIMEOUT = 5.0
WAIT_ERROR_TIMEOUT = {
    "gps_accuracy": 15.0 + DEFAULT_WAIT_ERROR_TIMEOUT,
    "network_strength_low_signal": 20.0 + DEFAULT_WAIT_ERROR_TIMEOUT,
    "network_strength_not_connected": 10.0 + DEFAULT_WAIT_ERROR_TIMEOUT,
}
WAIT_RUNNING_AFTER_RECOVERY_TIMEOUT = 30.0

POLL_PERIOD = 0.2
MOCK_RECONF_NODE = "/mock_state_signals_node"
ACKERMANN_SPEED_RUNNING = 1.0  # speed > this means controller is driving

# -----------------------------------------------------------------------------
# Mock config helpers
# -----------------------------------------------------------------------------


def good_config():
    return {
        "estop_enabled": False,
        "battery_level": 100,
        "gps_accuracy": 100.0,
        "signal_strength": SignalStrengthStamped.CONNECTED,
        "temperature": 30.0,
    }


def error_config_for_signal(signal_name):
    """Return dynamic_reconfigure dict that induces error for the given signal."""
    base = good_config()
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


# -----------------------------------------------------------------------------
# State / polling helpers
# -----------------------------------------------------------------------------


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


def get_ackermann_speed():
    try:
        msg = rospy.wait_for_message("/gem/ackermann_cmd", AckermannDrive, timeout=2.0)
        return getattr(msg, "speed", None)
    except rospy.ROSException:
        return None


def poll_until(condition, timeout_sec, desc="condition"):
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    while rospy.Time.now() < deadline:
        if condition():
            return True
        rospy.sleep(POLL_PERIOD)
    return False


# -----------------------------------------------------------------------------
# Rosbag control
# -----------------------------------------------------------------------------


def start_rosbag_record(bag_path):
    """Start rosbag record in background; returns Popen process. Caller must stop it."""
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


# -----------------------------------------------------------------------------
# Scenario base and common runner
# -----------------------------------------------------------------------------


class ScenarioBase(object):
    """
    Base for profiler scenarios. setup() runs before the rosbag is started;
    run() runs after start_rosbag_record() and ROSBAG_STARTUP_DELAY.
    """

    def setup(
        self,
        waypoints_file,
        serve_waypoints_proxy,
        cancel_tasks_proxy,
        signal_name,
        run_id,
    ):
        """
        Prepare system state before recording. Called before start_rosbag_record().
        Returns True if setup succeeded (so we can start the bag and run()).
        """
        raise NotImplementedError

    def run(self, signal_name, run_id):
        """
        Execute the scenario while the rosbag is recording (after ROSBAG_STARTUP_DELAY).
        Returns True if the scenario completed successfully.
        """
        raise NotImplementedError


def run_scenario_with_rosbag(
    scenario,
    test_dir,
    signal_name,
    run_id,
    waypoints_file,
    serve_waypoints_proxy,
    cancel_tasks_proxy,
):
    """
    Common runner: setup (before bag) -> start_rosbag_record -> startup_delay -> run -> stop_rosbag.
    Returns True if both setup and run succeeded.
    """
    subdir = os.path.join(test_dir, signal_name)
    os.makedirs(subdir, exist_ok=True)
    bag_path = os.path.join(subdir, "run_{:03d}.bag".format(run_id))

    if not scenario.setup(
        waypoints_file,
        serve_waypoints_proxy,
        cancel_tasks_proxy,
        signal_name,
        run_id,
    ):
        return False

    rosbag_proc = start_rosbag_record(bag_path)
    try:
        rospy.sleep(ROSBAG_STARTUP_DELAY)
        ret = scenario.run(signal_name, run_id)
        rospy.sleep(ROSBAG_DURATION_AFTER_ERROR)
        return ret
    finally:
        stop_rosbag_record(rosbag_proc)


# -----------------------------------------------------------------------------
# Scenario: Running -> Error
# -----------------------------------------------------------------------------


class RunToErrorScenario(ScenarioBase):
    """
    Running-to-error scenario. setup() gets system to IDLE then RUNNING.
    run() injects error, waits ERROR. (Runner sleeps ROSBAG_DURATION_AFTER_ERROR.)
    """

    def setup(
        self,
        waypoints_file,
        serve_waypoints_proxy,
        cancel_tasks_proxy,
        signal_name,
        run_id,
    ):
        set_mock_config(good_config())
        try:
            cancel_tasks_proxy(EmptyRequest())
        except rospy.ServiceException:
            pass
        rospy.sleep(0.5)

        if not poll_until(
            lambda: get_system_state() == SystemStateStamped.STATE_IDLE,
            WAIT_IDLE_TIMEOUT,
            "IDLE",
        ):
            rospy.logwarn("Run {} signal {}: did not reach IDLE in time".format(run_id, signal_name))
            return False

        try:
            resp = serve_waypoints_proxy(ServeWaypointsRequest(waypoints_file=waypoints_file))
            if not resp.success:
                rospy.logwarn("Run {} signal {}: serve_waypoints failed".format(run_id, signal_name))
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("Run {} signal {}: serve_waypoints error: {}".format(run_id, signal_name, e))
            return False

        if not poll_until(
            lambda: get_system_state() == SystemStateStamped.STATE_RUNNING
            and get_task_status() == TaskPlannerStatus.STATUS_RUNNING_TASK,
            WAIT_RUNNING_TIMEOUT,
            "RUNNING",
        ):
            rospy.logwarn("Run {} signal {}: did not reach RUNNING in time".format(run_id, signal_name))
            return False

        return True

    def run(self, signal_name, run_id):
        set_mock_config(error_config_for_signal(signal_name))
        poll_until(
            lambda: get_system_state() == SystemStateStamped.STATE_ERROR,
            WAIT_ERROR_TIMEOUT.get(signal_name, DEFAULT_WAIT_ERROR_TIMEOUT),
            "ERROR",
        )
        return True


# -----------------------------------------------------------------------------
# Scenario: Error -> Running (recovery)
# -----------------------------------------------------------------------------


class ErrorToRunScenario(ScenarioBase):
    """
    Error-to-running (recovery) scenario. setup() gets system to IDLE, RUNNING, then ERROR.
    run() clears error, waits full recovery. (Runner sleeps ROSBAG_DURATION_AFTER_ERROR.)
    """

    def setup(
        self,
        waypoints_file,
        serve_waypoints_proxy,
        cancel_tasks_proxy,
        signal_name,
        run_id,
    ):
        set_mock_config(good_config())
        try:
            cancel_tasks_proxy(EmptyRequest())
        except rospy.ServiceException:
            pass
        rospy.sleep(0.5)

        if not poll_until(
            lambda: get_system_state() == SystemStateStamped.STATE_IDLE,
            WAIT_IDLE_TIMEOUT,
            "IDLE",
        ):
            rospy.logwarn("Run {} signal {}: did not reach IDLE in time".format(run_id, signal_name))
            return False

        try:
            resp = serve_waypoints_proxy(ServeWaypointsRequest(waypoints_file=waypoints_file))
            if not resp.success:
                rospy.logwarn("Run {} signal {}: serve_waypoints failed".format(run_id, signal_name))
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("Run {} signal {}: serve_waypoints error: {}".format(run_id, signal_name, e))
            return False

        if not poll_until(
            lambda: get_system_state() == SystemStateStamped.STATE_RUNNING
            and get_task_status() == TaskPlannerStatus.STATUS_RUNNING_TASK,
            WAIT_RUNNING_TIMEOUT,
            "RUNNING",
        ):
            rospy.logwarn("Run {} signal {}: did not reach RUNNING in time".format(run_id, signal_name))
            return False

        set_mock_config(error_config_for_signal(signal_name))
        if not poll_until(
            lambda: get_system_state() == SystemStateStamped.STATE_ERROR,
            WAIT_ERROR_TIMEOUT.get(signal_name, DEFAULT_WAIT_ERROR_TIMEOUT),
            "ERROR",
        ):
            rospy.logwarn("Run {} signal {}: did not reach ERROR in time".format(run_id, signal_name))
            return False

        return True

    def run(self, signal_name, run_id):
        set_mock_config(good_config())

        def is_recovered():
            state_ok = get_system_state() == SystemStateStamped.STATE_RUNNING
            task_ok = get_task_status() == TaskPlannerStatus.STATUS_RUNNING_TASK
            return state_ok and task_ok

        if not poll_until(is_recovered, WAIT_RUNNING_AFTER_RECOVERY_TIMEOUT, "RUNNING after recovery"):
            rospy.logwarn("Run {} signal {}: did not recover to RUNNING in time".format(run_id, signal_name))
            return False

        return True


# -----------------------------------------------------------------------------
# Topic/predicate for analyzer: "good" value (error cleared) per signal
# -----------------------------------------------------------------------------
def _good_predicate_battery(msg):
    return getattr(msg, "battery_level", 0.0) > 0.5

def _good_predicate_estop(msg):
    return getattr(msg, "enabled", True) is False

def _good_predicate_temperature(msg):
    return getattr(msg, "temperature", 100.0) < 60.0

def _good_predicate_gps(msg):
    return getattr(msg, "accuracy", 999.0) <= 200.0

def _good_predicate_signal(msg):
    return getattr(msg, "strength", 0) == SignalStrengthStamped.CONNECTED

def _error_predicate_battery(msg):
    return getattr(msg, "battery_level", 1.0) <= 0.5

def _error_predicate_estop(msg):
    return getattr(msg, "enabled", True)

def _error_predicate_temperature(msg):
    return getattr(msg, "temperature", 0.0) >= 50.0

def _error_predicate_gps(msg):
    return getattr(msg, "accuracy", 0.0) > 200.0

def _error_predicate_signal_low(msg):
    return getattr(msg, "strength", 1) == 2  # LOW_SIGNAL

def _error_predicate_signal_not_connected(msg):
    return getattr(msg, "strength", 1) == 0  # NOT_CONNECTED

ERROR_TOPIC_AND_PREDICATE = {
    "battery_level": ("/battery_level", _error_predicate_battery),
    "estop": ("/estop", _error_predicate_estop),
    "temperature": ("/temperature", _error_predicate_temperature),
    "gps_accuracy": ("/gps_accuracy", _error_predicate_gps),
    "network_strength_low_signal": ("/signal_strength", _error_predicate_signal_low),
    "network_strength_not_connected": ("/signal_strength", _error_predicate_signal_not_connected),
}

GOOD_TOPIC_AND_PREDICATE = {
    "battery_level": ("/battery_level", _good_predicate_battery),
    "estop": ("/estop", _good_predicate_estop),
    "temperature": ("/temperature", _good_predicate_temperature),
    "gps_accuracy": ("/gps_accuracy", _good_predicate_gps),
    "network_strength_low_signal": ("/signal_strength", _good_predicate_signal),
    "network_strength_not_connected": ("/signal_strength", _good_predicate_signal),
}

# -----------------------------------------------------------------------------
# Scenario registry for CLI (name -> scenario instance for run_scenario_with_rosbag)
# -----------------------------------------------------------------------------
SCENARIO_RUN_TO_ERROR = "run_to_error"
SCENARIO_ERROR_TO_RUN = "error_to_run"

SCENARIOS = {
    SCENARIO_RUN_TO_ERROR: RunToErrorScenario(),
    SCENARIO_ERROR_TO_RUN: ErrorToRunScenario(),
}
