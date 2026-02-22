#!/usr/bin/env python3
"""
This module provides an abstract interface for path-tracking controllers running an
action server to accept waypoint navigation goals.

This abstract interface captures required ROS-specific features allowing controller implementations
to be developed in pure python (hence ROS-agnostic) through an abstract method stub (see control_step(...) below).

This module implements the following common functionalities for path tracking controllers:
- Fetches simulated state of the robot from gazebo
- Implements a control loop that executes control_step(...) implemented by controllers at a fixed frequency
- Instantiates and runs an action server to accept waypoint navigation goals and publishes feedback
- Publishes command velocities returned by control_step(...)
"""
from abc import ABC, abstractmethod
import numpy as np
from typing import NamedTuple, List

import rospy
import actionlib

from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose2D
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion

from gem_controller_action.msg import (
    NavigateWaypointsAction,
    NavigateWaypointsGoal,
    NavigateWaypointsFeedback,
    NavigateWaypointsResult
)


class RobotState(NamedTuple):
    """
    Represents robot state as pose and velocity of robot
    """
    x: float
    y: float
    x_dot: float
    y_dot: float
    yaw: float

class ControllerState(NamedTuple):
    """
    State representation for controller
    """
    # Desired control speed of robot
    speed: float
    # Desired control steering angle of robot
    steering_angle: float
    # Current waypoint index tracked by controller
    target_index: float
    # (Longitudinal) distance to target
    # Positive when target waypoint is ahead of robot, negative when behind
    # When distance is negative, we are considered to have crossed the waypoint
    dist_to_target: float

class ControllerActionInterface(ABC):
    def __init__(self, action_name = 'navigate_waypoints', controller_hz = 20.0):
        # Creates the action server but does not start immediately (see run())
        self._action_server = actionlib.SimpleActionServer(
            name=action_name,
            ActionSpec=NavigateWaypointsAction,
            execute_cb=self._execute_cb,
            auto_start=False
        )
        # Control loop frequency
        self._controller_hz = controller_hz
        # Publisher to set speed and steering angle of robot
        self._ackermann_pub = rospy.Publisher('/gem/ackermann_cmd', AckermannDrive, queue_size=1)

        rospy.loginfo("Waiting for gazebo get_model_state service to be up")
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.loginfo("Gazebo get_model_state service is up")

    def run(self):
        """
        Starts the action server
        """
        self._action_server.start()

    @staticmethod
    def get_gem_state() -> RobotState:
        """
        Gets the robot state (pose and velocity) from gazebo
        """      
        try:
            service_response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = service_response(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))

        x = model_state.pose.position.x
        y = model_state.pose.position.y

        orientation_q      = model_state.pose.orientation
        orientation_list   = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        x_dot = model_state.twist.linear.x
        y_dot = model_state.twist.linear.y

        return RobotState(round(x,4), round(y,4), round(x_dot,4), round(y_dot,4), round(yaw,4))

    def _execute_cb(self, goal: NavigateWaypointsGoal) -> None:
        """
        Action server execution callback for new navigation goals
        """
        control_rate = rospy.Rate(self._controller_hz)
        preempted = False
        done = False
        self.set_waypoints(goal.waypoints)
        # This index allows us to not backtrack to a previously completed waypoint
        next_waypoint_idx = goal.start_index

        while not done:
            # Preempt the goal if requested
            if self._action_server.is_preempt_requested():
                self._action_server.set_preempted()
                preempted = True
                break

            # Get the controller state. This includes the commands to send to the robot
            # and the target index and the longitudinal distance to that target index
            # The distance is positive if the target is ahead of the robot, otherwise negative
            controller_state = self.control_step(ControllerActionInterface.get_gem_state(), next_waypoint_idx)

            # Check if goal has been achieved by checking if the target index is the last waypoint
            # and the dist_to_target is negative (i.e. robot has crossed the last waypoint)
            next_waypoint_idx = controller_state.target_index
            if next_waypoint_idx == len(goal.waypoints) - 1 and controller_state.dist_to_target <= 0.:
                done = True

            # We publish the controller commands while the goal is not achieved
            if not done:
                self.publish_control_command(controller_state.speed, controller_state.steering_angle)

            # Publish feedback
            progress = (next_waypoint_idx + int(done)) / len(goal.waypoints)
            self.publish_feedback(next_waypoint_idx, progress)

            # Sleep until the next control step
            control_rate.sleep()

        # Stop the robot if we no longer have an active goal
        self.publish_control_command(0.0, 0.0)
        # If the navigation ended but was not preempted, set the result
        if not preempted:
            result = NavigateWaypointsResult()
            result.success = done
            if result.success:
                self._action_server.set_succeeded(result)
            else:
                self._action_server.set_aborted(result)

    def publish_control_command(self, speed: float, steering_angle: float) -> None:
        """
        Publishes driving command based on controller output
        """
        ackermann_msg = AckermannDrive()
        ackermann_msg.steering_angle_velocity = 0.0
        ackermann_msg.acceleration            = 0.0
        ackermann_msg.jerk                    = 0.0
        ackermann_msg.speed                   = speed
        ackermann_msg.steering_angle          = steering_angle
        self._ackermann_pub.publish(ackermann_msg)

    def publish_feedback(self, target_index: int, progress: float) -> None:
        """
        Publishes feedback for currently running goal
        """
        feedback = NavigateWaypointsFeedback()
        feedback.target_index = target_index
        feedback.progress = progress
        self._action_server.publish_feedback(feedback)

    def set_waypoints(self, poses: List[Pose2D]) -> None:
        """
        Set waypoints for controller to access for navigation 
        """
        self.path_points_x   = np.array([pose.x for pose in poses])
        self.path_points_y   = np.array([pose.y for pose in poses])
        self.path_points_yaw = np.array([pose.theta for pose in poses])
        self.dist_arr        = np.zeros(len(self.path_points_x))

    @abstractmethod
    def control_step(self, state: RobotState, start_idx: int) -> ControllerState:
        """
        Implement control step on the controllers implementing this class interface
        """
        pass

