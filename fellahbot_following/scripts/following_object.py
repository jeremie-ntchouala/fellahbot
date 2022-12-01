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
from fellahbot_following.image_processing import ImageProcessing
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class FollowingDriverROSWrapper:
    def __init__(self):

        rospy.Subscriber("speed_command", Int32, self.callback_speed_command)
        self._init_data_varaible()
        self._init_parameter_variable()
        self._init_service()
        self._init_publisher()
        self._init_camera_feed()
        self._init_controller_publisher()

    def _init_data_varaible(self, ):
        self.cv_image = None
        self.max_speed = 0
        self.publish_current_speed_frequency = 0
        self.publish_motor_status_frequency = 0

    def _init_parameter_variable(self, ):
        self.max_speed = rospy.get_param("~max_speed", 8)
        self.publish_current_speed_frequency = rospy.get_param("~publish_current_speed_frequency", 5.0)
        self.publish_motor_status_frequency = rospy.get_param("~publish_following_status_frequency", 1.0)
        self.controller = FollowingDriver(max_speed=self.max_speed)

    def _init_service(self, ):
        rospy.Service("stop_following", Trigger, self.callback_stop)

    def _init_publisher(self, ):
        self.current_speed_pub = rospy.Publisher("current_speed", Int32, queue_size=10)
        self.motor_status_pub = rospy.Publisher("following_status", DiagnosticStatus, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.publish_current_speed_frequency), self.publish_current_speed)
        rospy.Timer(rospy.Duration(1.0 / self.publish_motor_status_frequency), self.publish_motor_status)

    def _init_camera_feed(self, ):
        self.bridge = CvBridge()  # Creating an Instance of CV Bridge
        self.image_sub = rospy.Subscriber("/test1/camera1/image_raw", Image, self.image_callback)  # Subsciber for the Image feed

    def _init_controller_publisher(self, ):
        self.velocity_msg = Twist()  # Creating a messgae from the Twist template
        self.pub = rospy.Publisher('test1/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)  # Publisher to publish the velocities

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # self.controller.compute_control(self.cv_image) 
            self.control_loop()
        except CvBridgeError as e:
            print(e)
            self.controller.stop()

    def control_loop(self, ):
        # Creating an object of the class Image processing to process the incoming image
        sc = ImageProcessing()
        # process Image to detect object in the image
        result = sc.process_image(self.cv_image)
        x_length = result[0].shape[1]
        x = int(x_length/2)    # Center of the image
        # print(result[2])
        print('bounding box v1 : ', result[5])
        print('bounding box area : ', result[5][2] * result[5][3])
        # Showing the Original Frame and the Masked Frame
        cv2.imshow("Frame", result[0])
        mask3 = cv2.cvtColor(result[1], cv2.COLOR_GRAY2BGR)
        im_thresh_color = cv2.bitwise_and(result[0], mask3)
        cv2.line(im_thresh_color, (x, 0), (x, result[0].shape[1]), (255, 0, 0), 2)
        cv2.imshow("Mask", im_thresh_color)
        cv2.waitKey(1)

    def publish_current_speed(self, event=None):
        self.current_speed_pub.publish(self.controller.get_speed())

    def publish_motor_status(self, event=None):
        status = self.controller.get_status()
        data_list = []
        for key in status:
            data_list.append(KeyValue(key, str(status[key])))
        status_msg = DiagnosticStatus()
        status_msg.values = data_list
        self.motor_status_pub.publish(status_msg)

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
