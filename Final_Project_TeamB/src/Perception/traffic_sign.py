#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image


class Traffic_Sign():

    def __init__(self, img):
        self.img = img

        self.rect_x_left = 0
        self.rect_x_right = 0
        self.rect_y_top = 0
        self.rect_y_bottom = 0

    def convert_hsv(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        return hsv

    def convert_gray(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray

    def detect_light(self, img):
        hsv_img = self.convert_hsv(img)
        h, s, v = cv2.split(hsv_img)

        lower_white = np.array([0, 0, 180]) 
        upper_white = np.array([180, 255, 255])

        white_range = cv2.inRange(hsv_img, lower_white, upper_white)
        white_result = cv2.bitwise_and(v, v, mask=white_range)
        # dilation_image = cv2.dilate(white_result, (4,4), iterations=1)
        
        return white_result

    def detect_rect(self, img):
        gray_img = self.convert_gray(img)
        # cv2.imshow("gray", gray_img)
        ret, binary_img = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY) # 이진화
        binary_img = cv2.bitwise_not(binary_img)

        cv2.imshow('bin', binary_img)
        _, contours, _ = cv2.findContours(binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

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

            if 100 <= center[0] <= 300 and center[1] <= 130:
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                self.rect_x_left = x
                self.rect_x_right = x+w
                self.rect_y_top = y
                self.rect_y_bottom = y+h
        
        cv2.imshow('rect', img)

    def detect_greenlight(self):
        roi_img = self.img[:200, 160:480] # ROI
        blur_img = cv2.bilateralFilter(roi_img,9,75,75)
        cv2.imshow('original', roi_img)

        self.detect_rect(blur_img)
        if self.rect_x_left != 0 and self.rect_x_right != 0:
            light_img = self.detect_light(blur_img)

            circles = cv2.HoughCircles(light_img, cv2.HOUGH_GRADIENT, 1, 70, param1=60, param2=20, minRadius=10, maxRadius=30)
            if circles is not None:
                circles = np.uint16(np.around(circles))

                for i in circles[0,:]:
                    if (self.rect_x_right - (self.rect_x_right - self.rect_x_left)/3 < i[0]) and (self.rect_y_top < i[1] < self.rect_y_bottom):
                        cv2.circle(roi_img,(i[0],i[1]),i[2],(0,255,0),2)
                        cv2.circle(roi_img,(i[0],i[1]),2,(0,0,255),3)
                        return 1
                    cv2.circle(roi_img,(i[0],i[1]),i[2],(0,255,0),2)
                    cv2.circle(roi_img,(i[0],i[1]),2,(0,0,255),3)
        cv2.imshow('circle', roi_img)

        def __del__(self):
            print("checked traffic sign")
