#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math, time

class Hough():
    def __init__(self):
        self.width = 640
        self.height = 480
        self.offset = 350
        self.gap = 70

        self.lpos = 0
        self.rpos = 0

        # calibration config
        self.img_size = (640, 480)
        self.warp_img_w, self.warp_img_h, self.warp_img_mid = 640, 200, 100

        self.mtx = np.array([[ 358.78126,    0.     ,  318.15807],
            [0.     ,  360.19726,  234.48469],
            [0.     ,    0.     ,    1.     ]])
        self.dist = np.array([-0.325811, 0.079002, -0.000022, 0.001643, 0.000000])
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, self.img_size, 1, self.img_size)

        # perspective config
        self.warp_src  = np.array([[90, 330], [470, 330], 
                                   [20,  370], [620, 370]], dtype=np.float32)
        self.warp_dist = np.array([[0, 0], [540, 0],
                                   [0, 200], [640, 200]], dtype=np.float32)
        self.M = cv2.getPerspectiveTransform(self.warp_src, self.warp_dist)

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
        high_slope_threshold = 50

        # calculate slope & filtering with threshold
        slopes = []

        left_lines = []
        right_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                if x2 < self.width/2:
                    left_lines.append([x1, y1])
                    left_lines.append([x2, y2])
                else:
                    right_lines.append([x1, y1])
                    right_lines.append([x2, y2])
            else:
                slope = float(y2-y1) / float(x2-x1)
            
                if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
                    slopes.append(slope)
                    if slope < 0 and (x2 < self.width/2 + 90):
                        left_lines.append([x1, y1])
                        left_lines.append([x2, y2])
                    elif slope > 0 and (x1 > self.width/2 - 90):
                        right_lines.append([x1, y1])
                        right_lines.append([x2, y2])

        return left_lines, right_lines


    # get average m, b of lines
    def get_line_params(self, lines):
        x = np.array([lines[i][0] for i in range(len(lines))])
        y = np.array([lines[i][1] for i in range(len(lines))])
        fit = np.polyfit(y, x, 2)
        return fit

    # get lpos, rpos
    def get_line_pos(self, lines, left=False, right=False):
        if len(lines) == 0:
            fit = None
        else:
            fit = self.get_line_params(lines)

        x1, x2 = 0, 0
        ploty = np.linspace(0, 199, 200)
        if fit is None:
            if left:
                pos = self.lpos
            if right:
                pos = self.rpos
        else:
            fitx = fit[0]*ploty**2 + fit[1]*ploty + fit[2]
            x1 = fitx[0]
            x2 = fitx[-1]
            pos = fitx[50]

        return x1, x2, int(pos), fit

    def convert_gray(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray

    def convert_calibrated(self, img):
        calibrated_image = cv2.undistort(img, self.mtx, self.dist, None, self.cal_mtx)
        return calibrated_image

    def convert_perspective(self, img):
        return cv2.warpPerspective(img, self.M, (self.warp_img_w, self.warp_img_h), flags=cv2.INTER_LINEAR)

    def convert_bev(self, img, show=False):
        img = self.convert_calibrated(img)
        img = self.convert_perspective(img)
        return img

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
    def process_image(self, img, left=False, right=False):
        cv2.imshow("original", img)
        roi_img = self.convert_bev(img)
        cv2.imshow("bev", roi_img)

        blur_img = cv2.bilateralFilter(roi_img,9,75,75)
        blur_img = cv2.GaussianBlur(blur_img, (5, 5), 0)

        gray_img = self.convert_gray(blur_img)
        # cv2.imshow('blur', gray_img)

        # canny edge
        edge_img = cv2.Canny(np.uint8(gray_img), 60, 70)
        cv2.imshow("frame_roi", edge_img)
        
        all_lines = cv2.HoughLinesP(edge_img,1,math.pi/180,30,30,10)

        # divide left, right lines
        if all_lines is None:
            return (0, 640), img

        left_lines, right_lines = self.divide_left_right(all_lines)
        # # print(left_lines, right_lines)

        for i in left_lines:
            roi_img = cv2.line(roi_img, (i[0], i[1]), (i[0], i[1]), (0, 0, 255), 5)

        for i in right_lines:
            roi_img = cv2.line(roi_img, (i[0], i[1]), (i[0], i[1]), (0, 255, 0), 5)

        # cv2.imshow("roi", roi_img)

        # get center of lines
        if left:
            lx1, lx2, lpos, lfit = self.get_line_pos(left_lines, left=True)
            self.lpos = lpos
            if right == False:
                self.rpos = lpos + 250
        if right:
            rx1, rx2, rpos, rfit = self.get_line_pos(right_lines, right=True)
            self.rpos = rpos
            if left == False:
                self.lpos = rpos - 250

        if abs(self.rpos - self.lpos) < 200:
            if self.rpos < 240 or rx1 == 0:
                self.rpos = self.lpos + 300
            if self.lpos > 400 or lx1 == 0:
                self.lpos = self.rpos - 300

        print(self.lpos, self.rpos)
        # roi_img = cv2.line(roi_img, (int(lx1), 0), (int(lx2), 200), (255, 0,0), 3)
        # roi_img = cv2.line(roi_img, (int(rx1), 0), (int(rx2), 200), (255, 0,0), 3)
        
        # # draw rectangle
        roi_img = self.draw_rectangle(roi_img, self.lpos, self.rpos, offset=100)
        cv2.imshow('orig', roi_img)

        return (self.lpos, self.rpos), img
