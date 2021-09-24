#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2

class Traffic_Sign():

    def __init__(self):
        self.img = None

        self.rect_x_left = 0
        self.rect_x_right = 0
        self.rect_y_top = 0
        self.rect_y_bottom = 0

        self.traffic_start_count = 0
        self.traffic_flag = 0

    def convert_hsv(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        return hsv

    def convert_gray(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray

    def detect_light(self, img):
        hsv_img = self.convert_hsv(img)
        h, s, v = cv2.split(hsv_img)

        lower_white = np.array([0, 0, 100]) 
        upper_white = np.array([180, 255, 255])

        white_range = cv2.inRange(hsv_img, lower_white, upper_white)
        white_result = cv2.bitwise_and(v, v, mask=white_range)
        # dilation_image = cv2.dilate(white_result, (5,5), iterations=1)

        # cv2.imshow("wr", white_result)
        return white_result

    def detect_rect(self, img):
        gray_img = self.convert_gray(img)
        # cv2.imshow("gray", gray_img)

        ret, binary_img = cv2.threshold(gray_img, 80, 255, cv2.THRESH_BINARY) # 이진화
        binary_img = cv2.bitwise_not(binary_img)
 
        _, contours, _ = cv2.findContours(binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # for cnt in contours:
        #     cv2.drawContours(img, [cnt], 0, (255, 0, 0), 2)  # blue
        cv2.imshow("bi", binary_img)

        for cont in contours:
            length = cv2.arcLength(cont, True)
            area = cv2.contourArea(cont)
            
            if not ((area > 200) and (length > 200)):
                continue
            
            cont = cv2.convexHull(cont)
            cv2.drawContours(img, cont, 0, (0, 0, 255), 2)

            (x, y, w, h) = cv2.boundingRect(cont)

            center = (x + int(w/2), y + int(h/2))

            if 120 <= center[0] <= 250 and 50 <= center[1] <= 130:
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 3)
                self.rect_x_left = x
                self.rect_x_right = x+w
                self.rect_y_top = y
                self.rect_y_bottom = y+h
        
        cv2.imshow('rect', img)

    def first_roi(self, img):
        return img[0:200, 160:480]

    def second_roi(self, img):
        return img[100:200, 160:440]

    def detect_greenlight(self):
        if self.traffic_flag == 0:
            roi_img = self.first_roi(self.img) # ROI
        else:
            roi_img = self.second_roi(self.img)

        blur_img = cv2.GaussianBlur(roi_img, (5, 5), 0)
        cv2.imshow('original', roi_img)

        self.detect_rect(blur_img)
        
        print(self.rect_x_left, self.rect_x_right)
        if self.rect_x_left != 0 and self.rect_x_right != 0:
            light_img = self.detect_light(blur_img)
            
            circles = cv2.HoughCircles(light_img, cv2.HOUGH_GRADIENT, 1, 70, param1=60, param2=20, minRadius=10, maxRadius=30)
            if circles is not None:
                circles = np.uint16(np.around(circles))

                for i in circles[0,:]:
                    if (self.rect_x_right - (self.rect_x_right - self.rect_x_left)/3 < i[0]) and (self.rect_y_top < i[1] < self.rect_y_bottom):
                        cv2.circle(roi_img,(i[0],i[1]),i[2],(0,255,0),2)
                        cv2.circle(roi_img,(i[0],i[1]),2,(0,0,255),3)
                        print("green")
                        cv2.imshow('circle', roi_img)
                        return 1
                    cv2.circle(roi_img,(i[0],i[1]),i[2],(0,255,0),2)
                    cv2.circle(roi_img,(i[0],i[1]),2,(0,0,255),3)
        print("not green")
        cv2.imshow('circle', roi_img)
        return 0

        # def __del__(self):
        #     cv2.destroyAllWindows()
        #     print("checked traffic sign")
