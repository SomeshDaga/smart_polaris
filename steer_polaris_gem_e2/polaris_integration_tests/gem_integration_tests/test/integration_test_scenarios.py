#!/usr/bin/env python3
"""
Integration tests for state_manager, task_planner, and pure_pursuit with Gazebo/RViz (headless).

The goal of these tests is to ensure that navigation tasks to the controller only under safe conditions.

We employ the following comprehensive test scenarios to evaluate this goal:

Scenario 1: Issuing new navigation tasks in ERROR state

- Set system to ERROR state
- Send goal to task planner -> Planner should accept goal but not send to controller
- Undo the system ERROR state
- Planner should send goal to controller
- Controller executes plan with non-zero robot velocity

Scenario 2: Preempting active navigation tasks when system moves into ERROR state

- 
"""
import os
import sys
import rospy
import unittest
import rostest
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from ackermann_msgs.msg import AckermannDrive
from std_srvs.srv import Empty, EmptyRequest
from gem_state_msgs.msg import SystemStateStamped, TaskPlannerStatus, SignalStrengthStamped
from gem_controller_action.srv import ServeWaypoints, ServeWaypointsRequest
from dynamic_reconfigure.client import Client as DynReconfClient

import subprocess

PKG = "gem_integration_tests"
MOCK_RECONF_NODE = "/mock_state_signals_node"
WAYPOINTS_FILE_PARAM = "waypoints_file"


def _has_active_or_pending_goal(status_list):
    if not status_list:
        return False
    s = status_list[-1].status
    return s in (GoalStatus.PENDING, GoalStatus.ACTIVE)


class IntegrationTestScenarios(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("integration_test_scenarios", anonymous=False)
        rospy.sleep(5.)
        cls._waypoints_file = rospy.get_param(WAYPOINTS_FILE_PARAM)
        if not os.path.isabs(cls._waypoints_file):
            cls._waypoints_file = os.path.abspath(cls._waypoints_file)
        rospy.wait_for_service("/serve_waypoints", timeout=30)
        cls._serve_waypoints = rospy.ServiceProxy("/serve_waypoints", ServeWaypoints)
        cls._cancel_tasks = rospy.ServiceProxy("/cancel_tasks", Empty)

    @classmethod
    def tearDownClass(cls):
        # Kill gazebo since it can otherwise lead to hanging test suites
        # Forcefully kill gzserver
        subprocess.call(['pkill', '-9', 'gzserver'])

    def setUp(self):
        """
        Function called before each test case
        """
        # Apply good signals for the state manager
        self._set_good_mock_config()
        self.assertTrue(
            self._poll_until(
                lambda: self._get_system_state() == SystemStateStamped.STATE_IDLE,
                timeout_sec=1,
            ),
            "State manager did not transition to IDLE state",
        )

    def tearDown(self):
        # Cancel any current navigation tasks
        self._cancel_tasks(EmptyRequest())
        self.assertTrue(
            self._poll_until(
                lambda: self._get_task_status() == TaskPlannerStatus.STATUS_NO_TASK,
                timeout_sec=1,
            ),
            "Task planner did not transition to NO_TASK state",
        )

    def _set_good_mock_config(self):
        self._set_mock_config(estop_enabled=False, battery_level=100, gps_accuracy=100., signal_strength=SignalStrengthStamped.CONNECTED, temperature=40.)

    def _set_mock_config(self, estop_enabled=None, battery_level=None, gps_accuracy=None, signal_strength=None, temperature=None):
        """Update mock state signals via dynamic_reconfigure (partial update)."""
        client = DynReconfClient(MOCK_RECONF_NODE, timeout=5.0)
        config = client.get_configuration()
        if estop_enabled is not None:
            config["estop_enabled"] = estop_enabled
        if battery_level is not None:
            config["battery_level"] = battery_level
        if gps_accuracy is not None:
            config["gps_accuracy"] = gps_accuracy
        if signal_strength is not None:
            config["signal_strength"] = signal_strength
        if temperature is not None:
            config["temperature"] = temperature
        client.update_configuration(config)

    def _set_estop(self, enabled):
        self._set_mock_config(estop_enabled=enabled)

    def _wait_for_system_state(self, expected_state, timeout_sec=15.0):
        from gem_state_msgs.msg import SystemStateStamped
        msg = rospy.wait_for_message("/system_state", SystemStateStamped, timeout_sec)
        return msg.state == expected_state

    def _wait_for_task_status(self, expected_status, timeout_sec=15.0):
        msg = rospy.wait_for_message("/task_planner_status", TaskPlannerStatus, timeout_sec)
        return msg.status == expected_status

    def _poll_until(self, condition, timeout_sec=15.0, period=0.2):
        deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
        while rospy.Time.now() < deadline:
            if condition():
                return True
            rospy.sleep(period)
        return False

    def _get_system_state(self):
        try:
            msg = rospy.wait_for_message("/system_state", SystemStateStamped, timeout=2.0)
            return msg.state
        except rospy.ROSException:
            return None

    def _get_task_status(self):
        try:
            msg = rospy.wait_for_message("/task_planner_status", TaskPlannerStatus, timeout=2.0)
            return msg.status
        except rospy.ROSException:
            return None

    def _get_goal_status_list(self):
        try:
            msg = rospy.wait_for_message("/navigate_waypoints/status", GoalStatusArray, timeout=2.0)
            return msg.status_list
        except rospy.ROSException:
            return []

    def _get_ackermann_speed(self):
        try:
            msg = rospy.wait_for_message("/gem/ackermann_cmd", AckermannDrive, timeout=2.0)
            return msg.speed
        except rospy.ROSException:
            return None

    def test_01_new_navigation_task_during_error_state_and_starts_after_error_clears(self):
        """Scenario 1: New navigation task is issued while system in error. Should not start while system in error, starts immediately after error clears"""
        # Enable estop to put system in ERROR state
        self._set_estop(True)
        rospy.loginfo("Enabled ESTOP")
        self.assertTrue(
            self._poll_until(
                lambda: self._get_system_state() == SystemStateStamped.STATE_ERROR,
                timeout_sec=1,
            ),
            "System did not transition to ERROR with estop enabled",
        )
        rospy.loginfo("System in ERROR state")

        # Request navigation task to Task Planner
        resp = self._serve_waypoints(ServeWaypointsRequest(waypoints_file=self._waypoints_file))
        self.assertTrue(resp.success, "serve_waypoints should succeed (task accepted)")
        rospy.loginfo("Sent task to Task Planner")

        # Check if a navigation goal was sent to the controller (should not send in ERROR state)
        rospy.sleep(1.)
        status_list = self._get_goal_status_list()
        self.assertFalse(
            _has_active_or_pending_goal(status_list),
            "There must be no active or pending goal to navigate_waypoints when system state is ERROR",
        )
        rospy.loginfo("No goal sent to controller (expected since system in ERROR state)")

        # Check Task Planner has put task on hold/paused
        task_status = self._get_task_status()
        self.assertEqual(
            task_status,
            TaskPlannerStatus.STATUS_PAUSED_TASK,
            "task_planner_status must be STATUS_PAUSED_TASK",
        )
        rospy.loginfo("Task planner has paused task (expected since system in ERROR state)")

        # Reset the error state
        self._set_estop(False)
        rospy.loginfo("Disabled ESTOP")

        # Check system has moved to a RUNNING state
        self.assertTrue(
            self._poll_until(
                lambda: self._get_system_state() == SystemStateStamped.STATE_RUNNING,
                timeout_sec=1,
            ),
            "System did not transition to RUNNING with estop disabled",
        )
        rospy.loginfo("System in RUNNING state")

        # Check Task Planner has moved to a RUNNING state
        task_status = self._get_task_status()
        self.assertEqual(
            task_status,
            TaskPlannerStatus.STATUS_RUNNING_TASK,
            "task_planner_status must be STATUS_RUNNING_TASK",
        )
        rospy.loginfo("Task planner has resumed task")

        # Check controller has an active goal
        self.assertTrue(
            _has_active_or_pending_goal(self._get_goal_status_list()),
            "Controller must have gotten an updated goal"
        )
        rospy.loginfo("Controller goal has been reinstated")

        # Check robot velocity is non-zero
        rospy.sleep(1.)
        robot_speed = self._get_ackermann_speed()
        self.assertTrue(robot_speed > 0., "Robot speed must be non-zero")
        rospy.loginfo("Robot speed is non-zero i.e. running navigation")

    def test_02_ongoing_navigation_task_paused_on_error_condition_and_resumes_after_error_clears(self):
        """Scenario 2: Ongoing navigation task is paused when system encounters an error. Navigation task resumes after error clears"""
        # Request navigation task to Task Planner
        resp = self._serve_waypoints(ServeWaypointsRequest(waypoints_file=self._waypoints_file))
        self.assertTrue(resp.success, "serve_waypoints should succeed (task accepted)")
        rospy.loginfo("Sent task to Task Planner")

        # Check controller got an active goal
        self.assertTrue(
            self._poll_until(
                lambda: _has_active_or_pending_goal(self._get_goal_status_list()),
                timeout_sec=1,
            ),
            "Controller must have an active goal",
        )

        # Task planner state should be RUNNING
        task_status = self._get_task_status()
        self.assertEqual(
            task_status,
            TaskPlannerStatus.STATUS_RUNNING_TASK,
            "task_planner_status must be STATUS_RUNNING_TASK",
        )
        rospy.loginfo("Task planner has a RUNNING task")

        # System state should be RUNNING
        system_state = self._get_system_state()
        self.assertEqual(
            system_state,
            SystemStateStamped.STATE_RUNNING,
            "system_state must be STATE_RUNNING",
        )
        rospy.loginfo("System state is RUNNING")

        # Induce ERROR condition by enabling ESTOP
        self._set_estop(True)
        rospy.loginfo("Enabled ESTOP")

        # Check system in ERROR state
        self.assertTrue(
            self._poll_until(
                lambda: self._get_system_state() == SystemStateStamped.STATE_ERROR,
                timeout_sec=1,
            ),
            "System state did not transition to ERROR",
        )

        self.assertTrue(
            self._poll_until(
                lambda: self._get_task_status() == TaskPlannerStatus.STATUS_PAUSED_TASK,
                timeout_sec=1,
            ),
            "task_planner_status must be STATUS_PAUSED_TASK",
        )

        # Check goal for controller has been preempted
        self.assertFalse(
            _has_active_or_pending_goal(self._get_goal_status_list()),
            "Controller must not have an active goal"
        )
        rospy.loginfo("Controller goal has been preempted")

        # Remove the ESTOP condition
        self._set_estop(False)
        rospy.loginfo("Disabled ESTOP")

        # Check system has moved to a RUNNING state
        self.assertTrue(
            self._poll_until(
                lambda: self._get_system_state() == SystemStateStamped.STATE_RUNNING,
                timeout_sec=1,
            ),
            "System did not transition to RUNNING with estop disabled",
        )
        rospy.loginfo("System in RUNNING state")

        # Check Task Planner has moved to a RUNNING state
        self.assertTrue(
            self._poll_until(
                lambda: self._get_task_status() == TaskPlannerStatus.STATUS_RUNNING_TASK,
                timeout_sec=1,
            ),
            "task_planner_status must be STATUS_RUNNING_TASK",
        )
        rospy.loginfo("Task planner has resumed task")

        # Check controller has an active goal
        self.assertTrue(
            _has_active_or_pending_goal(self._get_goal_status_list()),
            "Controller must have gotten an updated goal"
        )
        rospy.loginfo("Controller goal has been reinstated")
    
        # Check robot velocity is non-zero
        rospy.sleep(1.)
        robot_speed = self._get_ackermann_speed()
        self.assertTrue(robot_speed > 0., "Robot speed must be non-zero")
        rospy.loginfo("Robot speed is non-zero i.e. running navigation")


if __name__ == "__main__":
    rostest.rosrun(PKG, "integration_test_scenarios", IntegrationTestScenarios, sysargs=sys.argv)
