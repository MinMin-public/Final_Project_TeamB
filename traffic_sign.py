#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)


class Traffic_Sign():

    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

        self.flag = 0
        self.traffic_x = 0
        self.traffic_x2 = 0
        self.traffic_y = 0
        self.traffic_y2 = 0

    def img_callback(self, img):
        self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")

    def transform_hsv(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        return hsv

    def transform_gray(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray

    def detect_traffic_light(self, img):
        hsv = self.to_hsv(img)
        h, s, v = cv2.split(hsv)

        lower_white = np.array([0, 0, 180]) 
        upper_white = np.array([180, 255, 255])

        white_range = cv2.inRange(hsv, lower_white, upper_white)
        white_result = cv2.bitwise_and(v, v, mask=white_range)
        #dilation_image = cv2.dilate(green_result, (4,4), iterations=1)
        
        # cv2.imshow("h", white_result)
        return white_result

    def detect_tf(self, img):
        img_gray = self.to_gray(img)
        ret, binary = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY) # 이진화
        binary = cv2.bitwise_not(binary)

        cv2.imshow('bin', binary)
        _, contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for cont in contours:
            length = cv2.arcLength(cont, True)
            area = cv2.contourArea(cont)

            if not ((area > 200) and (length > 200)):
                continue

            if len(cv2.approxPolyDP(cont, length*0.02, True)) != 4:
                continue

            (x, y, w, h) = cv2.boundingRect(cont)

            center = (x + int(w/2), y + int(h/2))
            _, width, _ = img.shape

            if 100 <= center[0] <= 300:
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                self.traffic_x = x
                self.traffic_x2 = x+w
                self.traffic_y = y
                self.traffic_y2 = y+h
        
        cv2.imshow('rect', img)

    def detect_circle(self):
        image = self.image[:200, 160:480] # ROI
        cv2.imshow('original', image)
        blur = cv2.bilateralFilter(image,9,75,75)
        self.detect_tf(blur)
        if self.traffic_x != 0 and self.traffic_x2 != 0:
            # print(self.traffic_x, self.traffic_x2)
            light_result = self.detect_traffic_light(blur)
            # img = cv2.medianBlur(hsv, 5)
            circles = cv2.HoughCircles(light_result, cv2.HOUGH_GRADIENT, 1, 40, param1=50, param2=20, minRadius=10, maxRadius=25)
            if circles is not None:
                circles = np.uint16(np.around(circles))

                for i in circles[0,:]:
                    # print(i)
                    # print(self.traffic_x, self.traffic_x2)
                    # print(self.traffic_x2 - (self.traffic_x2 - self.traffic_x)/3)
                    if (self.traffic_x2 - (self.traffic_x2 - self.traffic_x)/3) < i[0] and (self.traffic_y < i[1] < self.traffic_y2):
                        self.flag += 1
                        print("go")
                    cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
                    cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)

        cv2.imshow('circle', image)