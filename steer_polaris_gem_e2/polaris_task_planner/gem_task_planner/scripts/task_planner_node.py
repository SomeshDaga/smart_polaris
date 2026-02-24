#!/usr/bin/env python3
"""
This file provides a high-level Task Planner node that does the following:

- Exposes a /serve_waypoints service to load waypoints from a file and send to a controller for a new task
- Exposes a /task_planner_status topic to publish the status of the planner (indicating no/paused/running tasks)
- Sends navigation waypoints to a low-level controller (e.g. pure pursuit, stanley) using /navigate_waypoints action
- Pauses/Resumes tasks when system transitions in/out of ERROR states
"""
import os
import csv
from typing import NamedTuple, Optional, Tuple
from threading import Lock

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

from gem_state_msgs.msg import (
    SystemStateStamped,
    TaskPlannerStatus
)
from gem_controller_action.srv import (
    ServeWaypoints,
    ServeWaypointsRequest,
    ServeWaypointsResponse,
)
from gem_controller_action.msg import (
    NavigateWaypointsAction,
    NavigateWaypointsGoal,
    NavigateWaypointsFeedback,
    NavigateWaypointsResult
)


# Create an alias to the type that defines our task
Task = NavigateWaypointsGoal

class TaskPlannerNode:
    def __init__(self):
        # The task object represents the current task for the planner
        # This task may be in put in a paused or running state based on the
        # state of the robot
        self.task = None

        # Flag to indicate if current/new goals can be put into a running state
        self.allow_run_tasks = False

        # Cache the last system state to detect transitions in the system state
        # The system state is obtained via subscription from the state manager node
        self.last_system_state = None

        # Initialize action client to send waypoint goals to the controller
        # If a task is paused or cancelled, the goal will be preempted
        # If the task is resumed, the goal will be sent again to resume from the
        # last recorded progress from the controller
        self.client = actionlib.SimpleActionClient(
            'navigate_waypoints',
            NavigateWaypointsAction
        )

        # rospy callbacks (for subscribers, action clients and service servers) happen in different threads
        # we use locks to manage concurrent access to shared states e.g. self.task/self.client
        # for simplicity, we use a single lock for all shared resources
        self.lock = Lock()

        # Wait for the action server to become available
        rospy.loginfo("Waiting for navigate_waypoints action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to navigate_waypoints action server.")

        # Create a subscriber to the system state
        # This will be used to preeempt/resume active tasks
        self.system_state_sub = rospy.Subscriber(
            '/system_state',
            SystemStateStamped,
            self.system_state_callback
        )

        # Create publisher to advertise the task planner status
        self.planner_status_pub = rospy.Publisher(
            'task_planner_status',
            TaskPlannerStatus,
            queue_size=5
        )

        # Add a service to request serving of waypoints from a file
        # as a goal to the controller
        self.serve_waypoints_service = rospy.Service(
            'serve_waypoints',
            ServeWaypoints,
            self.handle_serve_waypoints
        )

        # Add another service to request cancellation of any pending/active task
        self.cancel_tasks_service = rospy.Service(
            'cancel_tasks',
            Empty,
            self.handle_cancel_tasks
        )

        # Allow auto-starting a goal on node startup if a ros param is set
        # with the waypoint file to use
        waypoint_file = rospy.get_param("~waypoint_file", None)
        if waypoint_file:
            rospy.loginfo("Got waypoint file: %s", waypoint_file)
            task = TaskPlannerNode.load_task_from_waypoint_file(waypoint_file)
            if task: self.start_task(task)
            else:
                rospy.logerr("Failed to load waypoints from file: %s", waypoint_file)
        else:
            rospy.loginfo("No waypoint file provided, will wait for serve_waypoints service to be called")

    @staticmethod
    def load_task_from_waypoint_file(waypoints_file: str) -> Optional[Task]:
        """
        Loads waypoints from a csv file to construct the task
        
        :param waypoints_file: Filename for waypoints file (relative to package://gem_task_planner/waypoints/)
        :return: Task object (represents goal to action server)
        """
        if not waypoints_file: return None

        if not os.path.isabs(waypoints_file):
            dirname  = os.path.dirname(__file__)
            waypoints_file = os.path.join(dirname, f'../waypoints/{waypoints_file}')

        task = NavigateWaypointsGoal()
        task.start_index = 0
        try:
            with open(waypoints_file) as f:
                poses = [tuple(line) for line in csv.reader(f)]
                for p in poses:
                    task.waypoints.append(Pose2D(float(p[0]), float(p[1]), float(p[2])))
        except Exception as e:
            print(f"Failed to load waypoints from file {filename}: {e}")
            task = None

        # If file was readable but contained no waypoints, we don't have a valid task
        return task if task and task.waypoints else None

    def pause_tasks(self) -> None:
        """
        Pauses the current task or any future tasks that replace the current task
        """
        self.allow_run_tasks = False
        if self.task and self.is_task_active():
            self.client.cancel_goal()
        # We not reset the current task

    def handle_cancel_tasks(self, req: EmptyRequest) -> EmptyResponse:
        with self.lock:
            self.cancel_task()
        return EmptyResponse()

    def cancel_task(self) -> None:
        """
        Cancels the current task and disables resumption of the task
        """
        if self.task and self.is_task_active():
            self.client.cancel_goal()
        self.task = None

    def resume_task(self) -> None:
        """
        Resumes the current task by starting the task from its current state
        """
        self.allow_run_tasks = True
        if self.task:
            return self.start_task(self.task)

    def start_task(self, task: Task) -> None:
        """
        Starts given task by canceling an active task (if any) and sending the task to the action server
        Does not send the task to the action server is running tasks is disabled but the task
        can be subsequently activated when resume_task() is called
        """
        # Cancel existing task (if any)
        rospy.loginfo(f"Starting new task. Last state: {self.last_system_state}")
        self.cancel_task()
        self.task = task
        # Send the new task to the controller (if the system is not in an error mode)
        # The task will be paused/resumed in the system_state_callback as the situation changes
        if self.allow_run_tasks:         
            rospy.loginfo(f"Sending nav goal: {self.last_system_state}")
            self.client.send_goal(
                self.task,
                done_cb=self.done_callback,
                active_cb=lambda: "Task successfully started",
                feedback_cb=self.feedback_callback
            )

    def system_state_callback(self, msg: SystemStateStamped) -> None:
        """
        Subscriber callback to /system_state to pause/resume tasks
        When the system transitions to an ERROR state, tasks are paused
        otherwise we resume tasks
        """
        with self.lock:
            state = msg.state

            if state != self.last_system_state:
                if state == SystemStateStamped.STATE_ERROR:
                    self.pause_tasks()
                else:
                    self.resume_task()

            self.last_system_state = state

    def handle_serve_waypoints(self, req: ServeWaypointsRequest) -> ServeWaypointsResponse:
        """
        Service callback to start a new task by loading waypoints from a file
        """
        with self.lock:
            task = TaskPlannerNode.load_task_from_waypoint_file(req.waypoints_file)
            if task: self.start_task(task)

            # Reply back with a response to the service call
            response = ServeWaypointsResponse()
            response.success = bool(task is not None)
            return response

    def feedback_callback(self, feedback: NavigateWaypointsFeedback) -> None:
        """
        Callback to process feedback from controller action server
        This is used to update the state of the current task
        In turn, this will allow to later resume a task from its latest state
        """
        with self.lock:
            if self.task:
                rospy.loginfo(f"Navigation progress: {int(feedback.progress * 100)}%")
                # Cache the currently tracked index for the list of waypoints
                # This is useful if we have to preempt the navigation goal due to system errors
                # and resume later
                self.task.start_index = feedback.target_index

    def done_callback(self, status: GoalStatus, result: NavigateWaypointsResult) -> None:
        """
        Callback to process done status from controller action server
        Clears the current task if the task is completed/aborted from the controller
        If the goal is preempted by this node (either to pause or replace with a new task),
        we delegate responsibility to clear or keep the task to the respective functions
        """
        # We do not explicitly reset the task status when a task is preempted
        # We leave that to the pause/cancel functions above since we have cancelled a task
        # to be resumed later or to be replaced by another task
        if status == GoalStatus.PREEMPTED:
            rospy.logwarn("Navigation was stopped")
            return

        with self.lock:
            self.task = None
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation succeeded")
        elif status == GoalStatus.ABORTED:
            rospy.logerr("Navigation aborted by controller")
        else:
            rospy.logwarn(f"Navigation ended with unexpected status: {status}")

    def is_task_active(self) -> bool:
        """
        Checks action server to see if a goal has been sent or is running
        :return: true if goal is sent/running, false otherwise
        """
        return self.client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]

    def publish_planner_status(self) -> None:
        """
        Publishes the status of the task planner to /task_planner_status
        """
        with self.lock:
            msg = TaskPlannerStatus()
            msg.header.stamp = rospy.Time.now()
            if self.task:
                if self.is_task_active():
                    msg.status = TaskPlannerStatus.STATUS_RUNNING_TASK
                else:
                    msg.status = TaskPlannerStatus.STATUS_PAUSED_TASK
            else:
                msg.status = TaskPlannerStatus.STATUS_NO_TASK
            self.planner_status_pub.publish(msg)

    def run(self):
        """
        Runs publish loop to publish the planner status at a fixed frequency
        """
        loop_rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.publish_planner_status()
            loop_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("task_planner")
    planner_node = TaskPlannerNode()
    planner_node.run()