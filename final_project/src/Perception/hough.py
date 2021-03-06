#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math, time

class Hough():
    def __init__(self):
        self.width = 640
        self.height = 480
        self.offset = 380
        self.gap = 50

        self.lpos = 0
        self.rpos = 0

    # draw lines
    def draw_lines(self, img, lines):
        for line in lines:
            x1, y1, x2, y2 = line[0]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            img = cv2.line(img, (x1, y1+self.offset), (x2, y2+self.offset), color, 2)
        return img

    # draw rectangle
    def draw_rectangle(self, img, lpos, rpos, offset=0):
        center = (lpos + rpos) / 2

        cv2.rectangle(img, (lpos - 5, 15 + offset),
                        (lpos + 5, 25 + offset),
                        (0, 255, 0), 2)
        cv2.rectangle(img, (rpos - 5, 15 + offset),
                        (rpos + 5, 25 + offset),
                        (0, 255, 0), 2)
        cv2.rectangle(img, (center-5, 15 + offset),
                        (center+5, 25 + offset),
                        (0, 255, 0), 2)    
        cv2.rectangle(img, (315, 15 + offset),
                        (325, 25 + offset),
                        (0, 0, 255), 2)
        return img

    # left lines, right lines
    def divide_left_right(self, lines):
        low_slope_threshold = 0
        high_slope_threshold = 20

        # calculate slope & filtering with threshold
        slopes = []

        left_lines = []
        right_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                slope = 0
            else:
                slope = float(y2-y1) / float(x2-x1)
            
            if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
                slopes.append(slope)
                if (slope < 0) and (x2 < self.width/2 - 90):
                    left_lines.append([x1, y1])
                    left_lines.append([x2, y2])
                elif (slope > 0) and (x1 > self.width/2 + 90):
                    right_lines.append([x1, y1])
                    right_lines.append([x2, y2])

        return left_lines, right_lines


    # get average m, b of lines
    def get_line_params(self, lines):
        x = np.array([lines[i][0] for i in range(len(lines))])
        y = np.array([lines[i][1] for i in range(len(lines))])

        A = np.vstack([x, np.ones(len(x))]).T

        m, b = np.linalg.lstsq(A, y)[0]

        return m, b

    # get lpos, rpos
    def get_line_pos(self, lines, left=False, right=False):
        if len(lines) == 0:
            m, b = 0, 0
        else:
            m, b = self.get_line_params(lines)
        
        print(m, b)
        x1, x2 = 0, 0
        if m == 0 and b == 0:
            if left:
                pos = self.lpos
            if right:
                pos = self.rpos
        else:
            y = self.gap / 2
            pos = (y - b) / m

            b += self.offset
            x1 = (self.height - b) / float(m)
            x2 = ((self.height/2) - b) / float(m)

        return x1, x2, int(pos)


    def convert_gray(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray

    def draw_steer(self, image, steer_angle):
        arrow_pic = cv2.imread('/home/minmin/xycar_ws/src/final_project/src/Control/steer_arrow.png', cv2.IMREAD_COLOR)

        origin_Height = arrow_pic.shape[0]
        origin_Width = arrow_pic.shape[1]
        steer_wheel_center = origin_Height * 0.74
        arrow_Height = self.height/2
        arrow_Width = (arrow_Height * 462)/728

        matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (steer_angle) * 2.5, 0.7)    
        arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height))
        arrow_pic = cv2.resize(arrow_pic, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

        gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

        arrow_roi = image[arrow_Height: self.height, (self.width/2 - arrow_Width/2) : (self.width/2 + arrow_Width/2)]
        arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
        res = cv2.add(arrow_roi, arrow_pic)
        image[(self.height - arrow_Height): self.height, (self.width/2 - arrow_Width/2): (self.width/2 + arrow_Width/2)] = res

        cv2.imshow('steer', image)

    # show image and return lpos, rpos
    def process_image(self, img):
        roi_img = img[self.offset : self.offset+self.gap, 0 : self.width]
        cv2.imshow("roi_img", roi_img)

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

        left_lines, right_lines = self.divide_left_right(all_lines)
        # print(left_lines, right_lines)

        for i in left_lines:
            roi_img = cv2.line(roi_img, (i[0], i[1]), (i[0], i[1]), (0, 0, 255), 5)

        for i in right_lines:
            roi_img = cv2.line(roi_img, (i[0], i[1]), (i[0], i[1]), (0, 0, 255), 5)

        cv2.imshow("roi", roi_img)

        # get center of lines
        lx1, lx2, lpos = self.get_line_pos(left_lines, left=True)
        rx1, rx2, rpos = self.get_line_pos(right_lines, right=True)

        self.lpos = lpos
        self.rpos = rpos 

        img = cv2.line(img, (int(lx1), self.height), (int(lx2), (self.height/2)), (255, 0,0), 3)
        img = cv2.line(img, (int(rx1), self.height), (int(rx2), (self.height/2)), (255, 0,0), 3)

        # roi_img = cv2.line(roi_img, (int(lx1), self.gap), (int(lx2), (0)), (255, 0,0), 3)
        # roi_img = cv2.line(roi_img, (int(rx1), self.gap), (int(rx2), (0)), (255, 0,0), 3)

        # draw lines
        # img = self.draw_lines(img, left_lines)
        # img = self.draw_lines(img, right_lines)
        # img = cv2.line(img, (230, 235), (410, 235), (255,255,255), 2)
        
        # # draw rectangle
        img = self.draw_rectangle(img, self.lpos, self.rpos, offset=self.offset)
        cv2.imshow('roi', img)

        return (lpos, rpos), img
