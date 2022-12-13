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
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class FollowingDriverROSWrapper():
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
        self.max_lin_vel = 3
        self.max_ang_vel = 0.5
        self.publish_current_speed_frequency = 0
        self.publish_motor_status_frequency = 0
        self.target_area = 25000
        self.target_center = 320

    def _init_parameter_variable(self, ):
        self.max_lin_vel = rospy.get_param("~max_lin_vel", 3)
        self.max_ang_vel = rospy.get_param("~max_ang_vel", 0.5)
        self.publish_current_speed_frequency = rospy.get_param("~publish_current_speed_frequency", 5.0)
        self.publish_motor_status_frequency = rospy.get_param("~publish_following_status_frequency", 1.0)
        self.controller = FollowingDriver(max_linear_speed=self.max_lin_vel, max_steering_speed=self.max_ang_vel, target= [self.target_area, self.target_center], Proportional=[0.00005, 0.003], Integrator=[0, 0.000005], Derivator=[0.000001, 0.001])

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
        self.cmd_vel_pub = rospy.Publisher('test1/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)  # Publisher to publish the velocities

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.control_loop()
        except CvBridgeError as e:
            print(e)
            self.stop()

    def control_loop(self, ):
        # Creating an object of the class Image processing to process the incoming image
        sc = ImageProcessing()
        # process Image to detect object in the image
        result = sc.process_image(self.cv_image)
        img_length = result[0].shape[1]
        length_mid = int(img_length/2)    # Center of the image
        current_area = 0
        current_center = 320
        
        print('bounding box center : ', result[2])
        print('bounding box area : ', result[6])
        # Showing the Original Frame and the Masked Frame

        try :
            rect_area = result[6]
            rect_center = result[2][0]
            if rect_area > current_area :
                current_area = rect_area
                current_center = rect_center

        except TypeError:
            self.stop()
            print("None type error receiving data")

        else:
            if current_area > 500:
                if (abs(self.target_area - current_area) < 1000):
                    current_area = 25000
                if (abs(self.target_center - current_center) < 15):
                    current_center = 320

                data = [current_area, current_center]
                self.controller.compute_control(data)

                cv2.imshow("Frame", result[0])
                mask3 = cv2.cvtColor(result[1], cv2.COLOR_GRAY2BGR)
                im_thresh_color = cv2.bitwise_and(result[0], mask3)
                cv2.putText(im_thresh_color, "Area = "+str(round(result[6], 2)), (
                    length_mid-140, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                cv2.line(im_thresh_color, (length_mid, 0),
                         (length_mid, result[0].shape[1]), (255, 0, 0), 2)
                cv2.imshow("Mask", im_thresh_color)
                cv2.waitKey(3)
        finally :
            self.send_command()

    def send_command(self, ):
        # Publish Twist command
        if self.controller.get_speed() < 0 : # Maintain direction straigh 
            self.velocity_msg.linear = Vector3(self.controller.get_speed(), 0.0, 0.0)
            self.velocity_msg.angular= Vector3(0.0, 0.0, -1*self.controller.get_steering())
        else :
            self.velocity_msg.linear = Vector3(self.controller.get_speed(), 0.0, 0.0)
            self.velocity_msg.angular= Vector3(0.0, 0.0, self.controller.get_steering())
        self.cmd_vel_pub.publish(self.velocity_msg)

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
