#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# @Project_name: catkin_ws
# @File_name:    controller_PID.py
# @User:         jeremie
# @Author:       Jeremie Ntchouala
# @Email:        jeremie.ntchouala@fellahtech.com
# @Time:         29 novembre 2022 at 16:41


from __future__ import print_function
from __future__ import division

import sys
import rospy
import time
import numpy as np


class ControllerPID:
    """very simple discrete PID controller"""

    def __init__(self, target, P, I, D):
        """
            Create a discrete PID controller
            each of the parameters may be a vector if they have the same length
            Args:
            target (double) -- the target value(s)
            P, I, D (double)-- the PID parameter
        """

        # check if parameter shapes are compatabile.
        if (not (np.size(P) == np.size(I) == np.size(D)) or ((np.size(target) == 1) and np.size(P) != 1) or
                (np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')

        rospy.loginfo('PID initialised with P:{}, I:{}, D:{}'.format(P, I, D))
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.setPoint = np.array(target)

        self.last_error = 0
        self.integrator = 0
        self.timeOfLastCall = None
    
    def __del__(self):
        """
        Close communication, and destroy all settings, ...
        """
        rospy.loginfo("Deleting the controller PID object")

    def update(self, current_value):
        """
            Updates the PID controller.
            Args:
                    current_value (double): vector/number of same legth as the target given in the constructor
            Returns:
                    controll signal (double): vector of same length as the target
        """
        # current states: [current_area, current_center]
        current_value = np.array(current_value)

        if np.size(current_value) != np.size(self.setPoint):
            raise TypeError('current_value and target do not have the same shape')

        if self.timeOfLastCall is None:
            # if the PID was called for the first time. we don't know the deltaT yet
            # no controll signal is applied
            self.timeOfLastCall = time.clock()
            return np.zeros(np.size(current_value))

        error = self.setPoint - current_value
        P = error

        currentTime = time.clock()
        deltaT = (currentTime - self.timeOfLastCall)

        # integral of the error is current error * time since last update
        self.integrator += error * deltaT
        I = self.integrator

        # derivative is difference in error / time since last update
        D = (error - self.last_error) / deltaT

        self.last_error = error
        self.timeOfLastCall = currentTime

        # return control signal
        return self.Kp * P + self.Ki * I + self.Kd * D


def main(args):
    test = ControllerPID()
    rospy.loginfo(test.get_status())
    try:
        rospy.spin()
    except:
        print("error")


if __name__ == '__main__':
    main(sys.argv)
