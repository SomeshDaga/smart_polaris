#!/usr/bin/env python3
"""
Publishes an OverlayText topic aggregating system state, task planner status,
action client status, and signal topics for display in RViz during integration tests.
"""
import rospy
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import ColorRGBA

from gem_controller_action.msg import (
    NavigateWaypointsActionFeedback
)

from gem_state_msgs.msg import (
    SystemStateStamped,
    TaskPlannerStatus,
    BatteryLevelStamped,
    GpsAccuracyStamped,
    SignalStrengthStamped,
    EstopStamped,
)
from jsk_rviz_plugins.msg import OverlayText

# Goal status strings for display
GOAL_STATUS_STR = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.LOST: "LOST",
}

SYSTEM_STATE_STR = {
    SystemStateStamped.STATE_IDLE: "IDLE",
    SystemStateStamped.STATE_RUNNING: "RUNNING",
    SystemStateStamped.STATE_ERROR: "ERROR",
}

TASK_STATUS_STR = {
    TaskPlannerStatus.STATUS_NO_TASK: "NO_TASK",
    TaskPlannerStatus.STATUS_PAUSED_TASK: "PAUSED_TASK",
    TaskPlannerStatus.STATUS_RUNNING_TASK: "RUNNING_TASK",
}

SIGNAL_STR = {
    SignalStrengthStamped.NOT_CONNECTED: "NOT_CONNECTED",
    SignalStrengthStamped.CONNECTED: "CONNECTED",
    SignalStrengthStamped.LOW_SIGNAL: "LOW_SIGNAL",
}


class StatusOverlayNode:
    def __init__(self):
        self._system_state = None
        self._task_status = None
        self._battery_level = None
        self._gps_accuracy = None
        self._signal_strength = None
        self._estop_enabled = None
        self._goal_status = "NO_GOAL"
        self._goal_progress = None
        self._ackermann_speed = None

        rospy.Subscriber("/system_state", SystemStateStamped, self._system_state_cb)
        rospy.Subscriber("/task_planner_status", TaskPlannerStatus, self._task_status_cb)
        rospy.Subscriber("/battery_level", BatteryLevelStamped, self._battery_cb)
        rospy.Subscriber("/gps_accuracy", GpsAccuracyStamped, self._gps_cb)
        rospy.Subscriber("/signal_strength", SignalStrengthStamped, self._signal_cb)
        rospy.Subscriber("/estop", EstopStamped, self._estop_cb)
        rospy.Subscriber("/navigate_waypoints/status", GoalStatusArray, self._goal_status_cb)
        rospy.Subscriber("/navigate_waypoints/feedback", NavigateWaypointsActionFeedback, self._goal_feedback_cb)
        rospy.Subscriber("/gem/ackermann_cmd", AckermannDrive, self._ackermann_cb)

        self._pub = rospy.Publisher(
            "/gem_integration_tests/status_overlay",
            OverlayText,
            queue_size=1,
        )
        self._rate = rospy.Rate(10)

    def _system_state_cb(self, msg):
        self._system_state = msg.state

    def _task_status_cb(self, msg):
        self._task_status = msg.status

    def _battery_cb(self, msg):
        self._battery_level = msg.battery_level

    def _gps_cb(self, msg):
        self._gps_accuracy = msg.accuracy

    def _signal_cb(self, msg):
        self._signal_strength = msg.strength

    def _estop_cb(self, msg):
        self._estop_enabled = msg.enabled

    def _goal_status_cb(self, msg):
        if msg.status_list:
            s = msg.status_list[-1]
            self._goal_status = GOAL_STATUS_STR.get(s.status, str(s.status))
        else:
            self._goal_status = "NO_GOAL"

    def _goal_feedback_cb(self, msg):
        self._goal_progress = msg.feedback.progress * 100.

    def _ackermann_cb(self, msg):
        self._ackermann_speed = msg.speed

    def _fmt(self, label, value):
        if value is None:
            return "%s: ---" % label
        return "%s: %s" % (label, value)

    def run(self):
        style = OverlayText()
        style.width = 320
        style.height = 340
        style.left = 10
        style.top = 10
        style.text_size = 12
        style.line_width = 2
        style.font = "DejaVu Sans Mono"
        style.fg_color = ColorRGBA(0.1, 1.0, 0.3, 1.0)
        style.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.7)

        while not rospy.is_shutdown():
            goal_progress_str = "N/A"
            if self._goal_progress is not None and self._task_status is not None:
                goal_progress_str = f"{str(int(self._goal_progress))}%" if self._task_status != TaskPlannerStatus.STATUS_NO_TASK else "N/A"
            state_str = SYSTEM_STATE_STR.get(self._system_state, str(self._system_state)) if self._system_state is not None else "N/A"
            task_str = TASK_STATUS_STR.get(self._task_status, str(self._task_status)) if self._task_status is not None else "N/A"
            sig_str = SIGNAL_STR.get(self._signal_strength, str(self._signal_strength)) if self._signal_strength is not None else "N/A"
            estop_str = "true" if self._estop_enabled else ("false" if self._estop_enabled is False else "N/A")
            speed_str = "%.2f" % self._ackermann_speed if self._ackermann_speed is not None else "N/A"

            style.text = (
                "--- navigate_waypoints action ---\n"
                "  Goal status: %s\n"
                "  Goal progress: %s\n"
                "--- /system_state ---\n"
                "  state: %s\n"
                "--- /task_planner_status ---\n"
                "  status: %s\n"
                "--- /battery_level ---\n"
                "  battery_level: %s\n"
                "--- /gps_accuracy ---\n"
                "  accuracy: %s\n"
                "--- /signal_strength ---\n"
                "  strength: %s\n"
                "--- /estop ---\n"
                "  enabled: %s\n"
                "--- /gem/ackermann_cmd ---\n"
                "  speed: %s\n"
            ) % (
                self._goal_status,
                goal_progress_str,
                state_str,
                task_str,
                "%.2f" % self._battery_level if self._battery_level is not None else "N/A",
                "%.1f" % self._gps_accuracy if self._gps_accuracy is not None else "N/A",
                sig_str,
                estop_str,
                speed_str,
            )
            self._pub.publish(style)
            self._rate.sleep()


def main():
    rospy.init_node("status_overlay_node", anonymous=False)
    node = StatusOverlayNode()
    node.run()


if __name__ == "__main__":
    main()
