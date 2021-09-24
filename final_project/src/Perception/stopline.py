#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math, time

class Stopline():
    def __init__(self):
        self.width = 640
        self.height = 480
        self.offset = 390
        self.gap = 40
        self.stopline_success = False

    # left lines, right lines
    def select_lines(self, lines):
        # calculate slope & filtering with threshold
        center_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            if y2 == y1:
                slope = 0
                center_lines.append([x1, y1, x2, y2])

        return center_lines

    def convert_gray(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray

    # show image and return lpos, rpos
    def detect_stopline(self, img):
        roi_img = img[self.offset : self.offset+self.gap, 160 : 450]
        # cv2.imshow("roi_img", roi_img)

        blur_img = cv2.bilateralFilter(roi_img,9,75,75)
        blur_img = cv2.GaussianBlur(blur_img, (5, 5), 0)

        gray_img = self.convert_gray(blur_img)
        # cv2.imshow('blur', gray_img)

        # canny edge
        edge_img = cv2.Canny(np.uint8(gray_img), 60, 70)
        # cv2.imshow("frame_roi", edge_img)
        
        all_lines = cv2.HoughLinesP(edge_img,1,math.pi/180,30,30,10)

        # divide left, right lines
        if all_lines is None:
            return (0, 640), img

        # print(all_lines)
        stoplines = self.select_lines(all_lines)
        print(stoplines)
        if len(stoplines) > 5:
            for i in stoplines:
                roi_img = cv2.line(roi_img, (i[0], i[1]), (i[2], i[3]), (0, 255, 0), 5)
            cv2.imshow("stopline", roi_img)
            self.stopline_success = True



