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
import cv2
from fellahbot_following.following_driver import FollowingDriver
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue


class FollowingDriverROSWrapper:
    def __init__(self):

        self.bridge = CvBridge() # Creating an Instance of CV Bridge
        self.image_sub =rospy.Subscriber("/test1/camera1/image_raw",Image,self.image_callback)  # Subsciber for the Image feed
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)                            # Publisher to publish the velocities
        self.velocity_msg = Twist()                                                             # Creating a messgae from the Twist template  
        

        max_speed = rospy.get_param("~max_speed", 8)
        publish_current_speed_frequency = rospy.get_param("~publish_current_speed_frequency", 5.0)
        publish_motor_status_frequency = rospy.get_param("~publish_following_status_frequency", 1.0)

        self.controller = FollowingDriver(max_speed=max_speed)
        rospy.Subscriber("speed_command", Int32, self.callback_speed_command)
        rospy.Service("stop_following", Trigger, self.callback_stop)

        # Publishing Speed and Status
        self.current_speed_pub = rospy.Publisher("current_speed", Int32, queue_size=10)
        self.motor_status_pub = rospy.Publisher("following_status", DiagnosticStatus, queue_size=1)
        rospy.Timer(rospy.Duration( 1.0/publish_current_speed_frequency), self.publish_current_speed)
        rospy.Timer(rospy.Duration( 1.0/publish_motor_status_frequency), self.publish_motor_status)

    def image_callback(self, data):  
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            self.controller.stop()


    def publish_current_speed(self, event=None):
        self.current_speed_pub.publish(self.controller.get_speed())

    def publish_motor_status(self, event=None):
        status = self.controller.get_status()
        data_list = []
        for key in status:
            data_list.append(KeyValue(key, str(status[key])))
        msg = DiagnosticStatus()
        msg.values = data_list
        self.motor_status_pub.publish(msg)

    def stop(self):
        self.controller.stop()

    def callback_speed_command(self, msg):
        self.controller.set_speed(msg.data)

    def callback_stop(self, req):
        self.stop()
        return {"success": True, "message": "Object following mode has been stopped"}


def main(args):
    rospy.init_node("following_object")
    object_following_wrapper = FollowingDriverROSWrapper()

    try:
        rospy.loginfo("Object following mode is now started, ready to track objects.")
        rospy.spin()
    except:
        print("error")

    rospy.on_shutdown(object_following_wrapper.stop)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
