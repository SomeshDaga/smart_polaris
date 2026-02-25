#!/usr/bin/env python3

#================================================================
# File name: pure_pursuit_sim.py                                                                  
# Description: pure pursuit controller for GEM vehicle in Gazebo (v2)                                                            
# Author: Hang Cui, Somesh Daga
# Email: hangcui3@illinois.edu, someshdaga94@gmail.com                                                                                                                                     
# Usage: rosrun gem_pure_pursuit_sim_v2 pure_pursuit_sim.py                                                                                                                       
#================================================================

# Python Headers
import math
import numpy as np
from numpy import linalg as la

# ROS Headers
import rospy

from gem_controller_action.controller_action_interface import ControllerActionInterface, ControllerState, RobotState

class PurePursuit(ControllerActionInterface):
    
    def __init__(self):
        super().__init__()
        self.look_ahead = 6    # meters
        self.wheelbase  = 1.75 # meters
        self.goal = 0

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    # find the angle bewtween two vectors
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    def control_step(self, state: RobotState, start_idx: int = 0) -> ControllerState:
        # get current position and orientation in the world frame
        curr_x, curr_y, curr_yaw = state.x, state.y, state.yaw

        # finding the distance of each waypoint from the current position
        # Note: someshdaga: We do not allow backtracking of target indexes since this can mess up when
        #                   doing waypoint navigation in circular loops and jump back to the first waypoint
        for i in range(start_idx, len(self.path_points_x)):
            self.dist_arr[i] = self.dist((self.path_points_x[i], self.path_points_y[i]), (curr_x, curr_y))

        # finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
        goal_arr = np.where( (self.dist_arr[start_idx:] < self.look_ahead + 0.3) & (self.dist_arr[start_idx:] > self.look_ahead - 0.3) )[0] + start_idx

        # finding the goal point which is the last in the set of points less than the lookahead distance
        for idx in goal_arr:
            v1 = [self.path_points_x[idx]-curr_x , self.path_points_y[idx]-curr_y]
            v2 = [np.cos(curr_yaw), np.sin(curr_yaw)]
            temp_angle = self.find_angle(v1,v2)
            if abs(temp_angle) < np.pi/2:
                self.goal = idx
                break

        # finding the distance between the goal point and the vehicle
        # true look-ahead distance between a waypoint and current position
        L = self.dist_arr[self.goal]

        # transforming the goal point into the vehicle coordinate frame 
        gvcx = self.path_points_x[self.goal] - curr_x
        gvcy = self.path_points_y[self.goal] - curr_y
        goal_x_veh_coord = gvcx*np.cos(curr_yaw) + gvcy*np.sin(curr_yaw)
        goal_y_veh_coord = gvcy*np.cos(curr_yaw) - gvcx*np.sin(curr_yaw)

        # find the curvature and the angle 
        alpha   = self.path_points_yaw[self.goal] - (curr_yaw)
        k       = 0.285
        angle_i = math.atan((2 * k * self.wheelbase * math.sin(alpha)) / L)
        angle   = angle_i*2
        angle   = round(np.clip(angle, -0.61, 0.61), 3)

        ct_error = round(np.sin(alpha) * L, 3)

        print("Crosstrack Error: " + str(ct_error))

        # implement constant pure pursuit controller
        return ControllerState(2.8, angle, self.goal, goal_x_veh_coord)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_sim_node', anonymous=True)
    pp = PurePursuit()
    pp.run()
    rospy.spin()


