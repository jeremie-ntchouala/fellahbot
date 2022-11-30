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
from fellahbot_following.controller_PID import ControllerPID


class FollowingDriver:
    def __init__(self, max_speed=10, max_steering=30):
        """
        Init communication, set default settings, ...
        """
        self.max_steering = max_steering
        self.max_speed = max_speed
        self.current_speed = 0
        self.current_steering = 0

        self.voltage = 48
        self.temperature = 37

    def __del__(self):
        """
        Close communication, and destroy all settings, ...
        """
        rospy.loginfo("Deleting the following driver object")

    def set_speed(self, speed):
        """
        Give a speed that the motor will try to reach.
        """
        if speed <= self.max_speed:
            self.current_speed = speed
        else:
            self.current_speed = self.max_speed

    def set_steering(self, steering):
        """
        Give a speed that the motor will try to reach.
        """
        if steering <= self.max_steering:
            self.current_steering = steering
        else:
            self.current_steering = self.max_steering

    def stop(self):
        """
        Set speed to 0 and thus stop the motor
        """
        self.current_speed = 0
        self.current_steering = 0
        rospy.loginfo("Stopping the driver controller")

    def get_speed(self):
        """
        Return current speed
        """
        return self.current_speed

    def get_steering(self):
        """
        Return current speed
        """
        return self.current_steering

    def get_status(self):
        """
        Get hardware information from the motor
        """
        return {
            'temperature': self.temperature,
            'voltage': self.voltage,
            'max_speed': self.max_speed,
            'max_steering': self.max_steering,
            'current_speed': self.get_speed(),
            'current_steering': self.get_steering(),
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
