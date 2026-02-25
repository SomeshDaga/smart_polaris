#!/usr/bin/env python3

#================================================================
# File name: stanley_sim.py                                                                  
# Description: stanley controller for GEM vehicle in Gazebo (v2)                                                        
# Author: Hang Cui, Somesh Daga
# Email: hangcui3@illinois.edu, someshdaga94@gmail.com                                                                    
# Usage: rosrun gem_stanley_sim stanley_sim.py                                                                    
#================================================================

# Python Headers
import math
import numpy as np
from numpy import linalg as la

# ROS Headers
import rospy

from gem_controller_action.controller_action_interface import ControllerActionInterface, ControllerState, RobotState

class Stanley(ControllerActionInterface):
    def __init__(self):
        super().__init__()
        self.wheelbase  = 1.75 # meters

    def pi_2_pi(self, angle: float) -> float:
        if angle > math.pi:
            return angle - 2.0 * math.pi
        if angle < -math.pi:
            return angle + 2.0 * math.pi

        return angle


    def control_step(self, state: RobotState, start_idx: int = 0) -> ControllerState:
        # get current position and orientation in the world frame
        # reference point is located at the center of rear axle
        curr_x, curr_y, curr_x_dot, curr_y_dot, curr_yaw = state

        # reference point is located at the center of frontal axle
        front_x = self.wheelbase*np.cos(curr_yaw) + curr_x
        front_y = self.wheelbase*np.sin(curr_yaw) + curr_y

        # find the closest point
        dx = [front_x - x for x in self.path_points_x]
        dy = [front_y - y for y in self.path_points_y]

        # find the index of closest point (do not allow backtracking by using the given start index)
        # Note: someshdaga: Fixed code to not allow backtracking by controller
        target_index = int(np.argmin(np.hypot(dx[start_idx:], dy[start_idx:]))) + start_idx

        front_axle_vec_rot_90 = np.array([[math.cos(curr_yaw - math.pi / 2.0)],
                                            [math.sin(curr_yaw - math.pi / 2.0)]])

        vec_target_2_front = np.array([[dx[target_index]], [dy[target_index]]])
        
        # crosstrack error
        ef = np.dot(vec_target_2_front.T, front_axle_vec_rot_90)
        ef = float(np.squeeze(ef))

        # vehicle heading 
        theta = curr_yaw

        # approximate heading of path at (path_x, path_y)
        # Note: someshdaga: Fixed out-of-bounds error with the target_index in the original code
        path_x      = self.path_points_x[target_index]
        path_y      = self.path_points_y[target_index]
        if target_index < len(self.path_points_x) - 1:
            path_x_next = self.path_points_x[target_index+1]
            path_y_next = self.path_points_y[target_index+1]
            theta_p     = np.arctan2(path_y_next-path_y, path_x_next-path_x)
        else:
            theta_p = 0.

        # theta_e is the heading error
        theta_e = self.pi_2_pi(theta_p-theta)

        f_vel = round(np.sqrt(curr_x_dot**2 + curr_y_dot**2), 3)

        delta = round(theta_e + math.atan2(0.45 * ef, f_vel), 3)

        theta_e  = round(np.degrees(theta_e), 1)

        ef = round(ef,3)
        print("Crosstrack Error: " + str(ef) + ", Heading Error: " + str(theta_e))

        # Calculate longitudinal distance to the target index
        gvcx = self.path_points_x[target_index] - curr_x
        gvcy = self.path_points_y[target_index] - curr_y
        goal_x_veh_coord = gvcx*np.cos(curr_yaw) + gvcy*np.sin(curr_yaw)

        # implement constant stanley controller
        return ControllerState(2.8, delta, target_index, goal_x_veh_coord)


if __name__ == '__main__':
    rospy.init_node('stanley_sim_node', anonymous=True)
    sl = Stanley()
    sl.run()
    rospy.spin()

