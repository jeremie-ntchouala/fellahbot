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


class ImageProcessing:
    def __init__(self, lower=(29, 86, 6), upper=(64, 255, 255)):
        self.lower = lower  # The lower limit of the colour range in the form of RGB Values
        self.upper = upper  # The Upper limit of the colour range in the form of RGB Values
        self.radius = None  # The radius of the Circle drawn around the object
        self.center = None  # The center of the circle drawn around the object

    def bounding_box(self, contours):
        box = []
        peri = cv2.arcLength(contours, True)
        approx = cv2.approxPolyDP(contours, 0.04 * peri, True)  # 0.05 or 0.04
        if len(approx) == 4:
            target = approx
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
            c = max(cnts, key=cv2.contourArea)

            # Bounding rectangle version 1
            bbox = self.bounding_box(c)
            # Bounding rectangle version 2
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            ((x, y), self.radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if self.radius > 10:
                cv2.circle(self.frame, (int(x), int(y)), int(self.radius), (0, 255, 255), 2)
                cv2.circle(frame, self.center, 5, (0, 0, 255), -1)
                # cv2.drawContours(frame, [box], -1, (255, 0, 255), 1)
                cv2.rectangle(frame, (bbox[0], bbox[1]), ((bbox[0]+bbox[2]),(bbox[1]+bbox[3])), (255, 0, 255), 1)
                

        return [frame, mask, self.center, self.radius, bbox, box]


def main(args):
    image = cv2.imread("/home/jeremie/Frame_3.png")
    # while 1:
    sc = ImageProcessing((29, 86, 6), (64, 255, 255))
    result = sc.process_image(image)
    cv2.imshow("Frame", result[0])
    cv2.imshow("Mask", result[1])
    cv2.waitKey(1)
    print('center ', result[2])
    print('radius ', result[3])
    print('bounding box v1 ', result[4])
    print(result[0].shape)
    print('bounding box v2', result[5])

    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        pass
        #break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
