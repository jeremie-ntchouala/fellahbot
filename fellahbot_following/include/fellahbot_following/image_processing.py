#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# @Project_name: catkin_ws
# @File_name:    image_processing.py
# @User:         jeremie
# @Author:       Jeremie Ntchouala
# @Email:        jeremie.ntchouala@fellahtech.com
# @Time:         29 novembre 2022 at 17:58

from __future__ import print_function
from __future__ import division

import sys
import cv2
import imutils
import numpy as np


class ImageProcessing():
    def __init__(self, lower=(29, 86, 6), upper=(64, 255, 255)):
        self.lower = lower  # The lower limit of the colour range in the form of RGB Values
        self.upper = upper  # The Upper limit of the colour range in the form of RGB Values
        self.radius = None  # The radius of the Circle drawn around the object
        self.center = None  # The center of the circle drawn around the object
        self.area = None    # The area of the box drawn around the object
        self.frame = None

    def bounding_box(self, contours):
        box = []
        peri = cv2.arcLength(contours, True)
        approx = cv2.approxPolyDP(contours, 0.004 * peri, True)  # 0.05 or 0.04
        box = cv2.boundingRect(approx)

        return box

    def rect_to_point(self, box):
        return

    def process_image(self, frame):
        self.frame = frame
        
        blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)

        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower, self.upper)

        mask = cv2.erode(mask, None, iterations=2)

        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cnts = imutils.grab_contours(cnts)

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)  # Maximum countour area

            # Bounding rectangle version 1
            bbox = self.bounding_box(c)
            # Bounding rectangle version 2
            rect = cv2.minAreaRect(c)
            rect_width = rect[1][0]
            rect_height = rect[1][1]
            self.area = np.int0(rect_width * rect_height)
            box = cv2.boxPoints(rect).astype('int')
            # box = np.int0(box)

            ((x, y), self.radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if np.int0(M["m00"]) != 0:
                self.center = (np.int0(M["m10"] / M["m00"]),
                               np.int0(M["m01"] / M["m00"]))
            else:
                self.center = (np.int0(x), np.int0(y))

            if self.radius > 10:
                cv2.circle(self.frame, (np.int0(x), np.int0(y)),np.int0(self.radius), (0, 255, 255), 2)
                cv2.circle(frame, self.center, 5, (0, 0, 255), -1)
                cv2.drawContours(frame, [box], -1, (255, 0, 255), 1)
                # cv2.rectangle(frame, (bbox[0], bbox[1]), ((bbox[0]+bbox[2]),(bbox[1]+bbox[3])), (255, 0, 255), 1)
        else:
            bbox, box = [], []
            self.center = None 
            self.radius = None
            self.area = None  

        return [frame, mask, self.center, self.radius, bbox, box, self.area]


def main(args):
    image = cv2.imread("/home/jeremie/Frame_3.png")

    object_detector = cv2.createBackgroundSubtractorMOG2()

    sc = ImageProcessing((29, 86, 6), (64, 255, 255))
    result = sc.process_image(image)
    mask = object_detector.apply(image)
    # while 1:
    cv2.imshow("Mask background", mask)
    cv2.imshow("Frame", result[0])
    cv2.imshow("Mask", result[1])
    cv2.waitKey(1)
    print('center ', result[2])
    print('radius ', result[3])
    print('bounding box v1 ', result[4])
    print(result[0].shape)
    print('bounding box v2', result[5])
    print('Box Area :', result[6])
    print('center :', result[2])

    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        pass
    # break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
