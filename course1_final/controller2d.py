#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np
import math

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        curWaypoint_ID = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
            curWaypoint_ID = min_idx
        else:
            desired_speed = self._waypoints[-1][2]
            curWaypoint_ID = -1
        self._desired_speed = desired_speed

        return curWaypoint_ID

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def find_waypointWithMinDist(self, waypointID, l=8.5):
        result = waypointID

        curDist = np.linalg.norm(np.array([
                    self._waypoints[waypointID][0] - self._current_x,
                    self._waypoints[waypointID][1] - self._current_y]))

        while curDist < l:
            result += 1
            curDist = np.linalg.norm(np.array([
                self._waypoints[result][0] - self._current_x,
                self._waypoints[result][1] - self._current_y]))

            if result >= len(self._waypoints)-1:
                result = len(self._waypoints)-1
                break

        return result

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        waypointID      = self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_diff_now', 0.0)
        self.vars.create_var('v_diff_old', 0.0)
        self.vars.create_var('e_l_now', 0.0)
        self.vars.create_var('e_l_old', 0.0)

        self.vars.v_diff_old = 0.0
        self.vars.e_l_old = 0.0



        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            # # the simplest throttle command I can imagine
            # if (v-v_desired) < 0:
            #     throttle_output = 1
            # else:
            #     throttle_output = 0

            K_p = 0.10
            K_i = 0.050
            K_d = 0.050
            dt = 0.033

            self.vars.v_diff_now = v_desired - v

            acc =   K_p * self.vars.v_diff_now \
                  + K_d * (self.vars.v_diff_now - self.vars.v_diff_old) / dt \
                  + K_i * 0.5 * (self.vars.v_diff_now + self.vars.v_diff_old) * dt

            if acc < 0:
                break_output = -acc
                throttle_output = 0
            else:
                break_output = 0
                throttle_output = acc

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            # throttle_output = 1
            # brake_output    = 0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            '''
            # first try with pure pursuit (35%)
            ###################################
            '''
            # alpha_hat = math.atan2(y_w - y, x_w - x)
            # alpha = alpha_hat - yaw
            # L = 1
            # k = 0.8
            # k_s = 0.01
            # delta = math.atan(2*L*math.sin(alpha)/(k_s + k*v))

            '''
            # second try with Stanley (63.5%)
            #################################
            '''
            # x_w = waypoints[waypointID][0]
            # y_w = waypoints[waypointID][1]
            #
            # e = math.sqrt((x_w-x)*(x_w-x) + (y_w-y)*(y_w-y))
            # psi = 0
            #
            # if waypointID < len(waypoints)-1:
            #     waypointDir = math.atan2((waypoints[waypointID+1][1] - y_w), (waypoints[waypointID+1][0] - x_w))
            #     psi = yaw - waypointDir
            # else:
            #     pass
            #
            # k = 0.03
            # k_s = 0.0001
            #
            # delta = -psi + math.atan2((k * e), (k_s + v))
            #
            # steer_output = delta

            '''
            # third try with online help (100%)
            ###################################
            '''
            NEWwaypointID = self.find_waypointWithMinDist(waypointID, l=8.5)

            x_w = waypoints[NEWwaypointID][0]
            y_w = waypoints[NEWwaypointID][1]
            self.vars.e_l_now = -(x_w - x) * np.sin(yaw) + (y_w - y) * np.cos(yaw)

            Kp_l = 0.05
            Ki_l = 0.08
            Kd_l = 0.08
            alpha_l = 0.03

            acc_l =   Kp_l * self.vars.e_l_now \
                    + Ki_l * 0.5 * (self.vars.e_l_now + self.vars.e_l_old) * dt \
                    + Kd_l * (self.vars.e_l_now - self.vars.e_l_old) / dt

            steer_output = 1.22 * np.tanh(alpha_l * acc_l)

            '''
            # fourth try - updated Stanley with what I learned online (100%)
            ################################################################
            new special point: make look-ahead distance dependent on velocity!
            '''
            NEWwaypointID = self.find_waypointWithMinDist(waypointID, l=0.5*v+0.5)
            x_w = waypoints[NEWwaypointID][0]
            y_w = waypoints[NEWwaypointID][1]

            e = -(x_w - x) * np.sin(yaw) + (y_w - y) * np.cos(yaw)
            psi = 0

            if waypointID < len(waypoints)-10:
                waypointDir = math.atan2((waypoints[NEWwaypointID][1] - waypoints[waypointID][1]),
                                         (waypoints[NEWwaypointID][0] - waypoints[waypointID][0]))
                psi = yaw - waypointDir
            else:
                pass

            k = 0.03
            k_s = 0.01

            delta = -psi + math.atan2((k * e), (k_s + v))

            steer_output = 1.22 * delta

            # Change the steer output with the lateral controller. 
            # steer_output    = 0

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        # self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.v_diff_old = self.vars.v_diff_now
