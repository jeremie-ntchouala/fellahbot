#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# @Project_name: catkin_ws
# @File_name:    following_driver.py
# @User:         jeremie
# @Author:       Jeremie Ntchouala
# @Email:        jeremie.ntchouala@fellahtech.com
# @Time:         23 novembre 2022 at 16:41

from __future__ import print_function
from __future__ import division

import sys
import rospy
import numpy as np
from fellahbot_following.controller_PID import ControllerPID
from fellahbot_following.image_processing import ImageProcessing


class FollowingDriver():
    def __init__(self, max_linear_speed=3, max_steering_speed=0.5, target=[25000, 320], Proportional=None, Integrator=None, Derivator=None ):
        """
        Init communication, set default settings, ...
        """
        self.max_linear_speed = max_linear_speed
        self.max_steering_speed = max_steering_speed
        self.current_linear_speed = 0
        self.current_steering_speed = 0

        self.Proportional = Proportional
        self.Integrator = Integrator
        self.Derivator = Derivator
        self.target_area , self.target_center = target

        # ControllerPID.__init__(target, Proportional, Integrator, Derivator)
        self.PID_controller = ControllerPID(target, Proportional, Integrator, Derivator)

    def __del__(self):
        """
        Close communication, and destroy all settings, ...
        """
        rospy.loginfo("Deleting the following driver object")

    def compute_control(self, info):
        [uncliped_lin_speed, uncliped_ang_speed] = self.PID_controller.update(info)
        # clip these speeds to be less then the maximal speed specified above
        linear_vel  = np.clip(uncliped_lin_speed, -self.max_linear_speed, self.max_linear_speed)
        angular_vel = np.clip(uncliped_ang_speed, -self.max_steering_speed, self.max_steering_speed)
        self.set_speed(linear_vel)
        self.set_steering(angular_vel)

    def set_speed(self, speed):
        """
        Give a speed that the motor will try to reach.
        """
        if abs(speed) <= self.max_linear_speed:
            self.current_linear_speed = speed
        else:
            self.current_linear_speed = self.max_linear_speed

    def set_steering(self, steering):
        """
        Give a speed that the motor will try to reach.
        """
        if abs(steering) <= self.max_steering_speed:
            self.current_steering_speed = steering
        else:
            self.current_steering_speed = self.max_steering_speed

    def stop(self):
        """
        Set speed to 0 and thus stop the motor
        """
        self.current_linear_speed = 0
        self.current_steering_speed = 0
        rospy.loginfo("Stopping the driver controller")

    def get_speed(self):
        """
        Return current speed
        """
        return self.current_linear_speed

    def get_steering(self):
        """
        Return current speed
        """
        return self.current_steering_speed

    def get_status(self):
        """
        Get hardware information from the motor
        """
        return {
            'PID param': 'P:{}, I:{}, D:{}'.format(self.Proportional, self.Integrator, self.Derivator), 
            'max_linear_speed': self.max_linear_speed,
            'max_steering_speed': self.max_steering_speed,
            'current_linear_speed': self.get_speed(),
            'current_steering_speed': self.get_steering(),
            'target_center':  self.target_area,
            'target_center': self.target_center,

        }


def main(args):
    test = FollowingDriver()
    rospy.loginfo(test.get_status())
    try:
        rospy.spin()

    except:
        print("error")


if __name__ == '__main__':
    main(sys.argv)
