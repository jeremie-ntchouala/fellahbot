#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# @Project_name: catkin_ws
# @File_name:    following_object.py
# @User:         jeremie
# @Author:       Jeremie Ntchouala
# @Email:        jeremie.ntchouala@fellahtech.com
# @Time:         23 novembre 2022 at 16:41

from __future__ import print_function
from __future__ import division

import sys


import rospy
from modules.following_driver import FollowingDriver
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue


def main(args):
    pass


if __name__ == '__main__':
    main(sys.argv)
