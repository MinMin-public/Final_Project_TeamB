#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math, time, collections
from cv_bridge import CvBridge
#from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
#from PID import PID
import sys
import os
import signal
import copy


def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

class LaneDetection():
    def __init__(self):
        self.image = np.empty(shape=[0])
        self.bridge = CvBridge()
        self.pub = None
        self.Width = 640
        self.Height = 480
        self.Offset = 410
        self.Gap = 50
        self.rpos_deque = collections.deque([])
        self.lpos_deque = collections.deque([])

        rospy.init_node('auto_drive')
        #pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

    def img_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def ROI(self, frame, vertices):
        # blank mask:
        mask = np.zeros_like(frame)
        # fill the mask
        cv2.fillPoly(mask, vertices, 255)

        # now only show the area that is the mask
        masked = cv2.bitwise_and(frame, mask)
        return masked

    # draw lines
    def draw_lines(self, img, lines):
        for line in lines:
            x1, y1, x2, y2 = line[0]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            img = cv2.line(img, (x1, y1 + self.Offset), (x2, y2 + self.Offset), color, 2)
        return img


    # draw rectangle
    def draw_rectangle(self, img, lpos, rpos):
        center = (lpos + rpos) / 2

        cv2.rectangle(img, (lpos - 5, self.Gap/2 - 5 + self.Offset),
                    (lpos + 5, self.Gap/2 + 5 + self.Offset),
                    (0, 255, 0), 2)
        cv2.rectangle(img, (rpos - 5, self.Gap/2 - 5 + self.Offset),
                    (rpos + 5, self.Gap/2 + 5 + self.Offset),
                    (0, 255, 0), 2)
        cv2.rectangle(img, (center - 5, self.Gap/2 - 5 + self.Offset),
                    (center + 5, self.Gap/2 + 5 + self.Offset),
                    (0, 255, 0), 2)
        cv2.rectangle(img, (self.Width/2 - 5, self.Gap/2 - 5 + self.Offset),
                    (self.Width/2 + 5, self.Gap/2 + 5 + self.Offset),
                    (0, 0, 255), 2)
        return img


    # def divide_left_right_curve(self, lines):

    #     low_slope_threshold = 0
    #     high_slope_threshold = 10

    #     # calculate slope & filtering with threshold
    #     slopes = []
    #     new_lines = []

    #     for line in lines:
    #         x1, y1, x2, y2 = line[0]

    #         if x2 - x1 == 0:
    #             slope = 0
    #         else:
    #             slope = float(y2 - y1) / float(x2 - x1)

    #         if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
    #             slopes.append(slope)
    #             new_lines.append(line[0])

    #     # divide lines left to right
    #     left_lines = []
    #     right_lines = []

    #     for j in range(len(slopes)):
    #         Line = new_lines[j]
    #         slope = slopes[j]

    #         x1, y1, x2, y2 = Line

    #         if (slope < 0) and (x2 < self.Width / 2 - 40):
    #             left_lines.append([Line.tolist()])
    #         elif (slope > 0) and (x1 > self.Width / 2 + 40):
    #             right_lines.append([Line.tolist()])

    #     return left_lines, right_lines

    # left lines, right lines
    def divide_left_right(self, lines):
        low_slope_threshold = 0
        high_slope_threshold = 30

        # calculate slope & filtering with threshold
        slopes = []
        new_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                slope = 0
            else:
                slope = float(y2 - y1) / float(x2 - x1)

            if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
                slopes.append(slope)
                new_lines.append(line[0])

        # divide lines left to right
        left_lines = []
        right_lines = []

        for j in range(len(slopes)):
            Line = new_lines[j]
            slope = slopes[j]

            x1, y1, x2, y2 = Line

            if (slope < 0) and (x2 < self.Width - 30):
                left_lines.append([Line.tolist()])
            elif (slope > 0) and (x1 > 30):
                right_lines.append([Line.tolist()])

        return left_lines, right_lines

    # get average m, b of lines
    def get_line_params(self, lines):
        # sum of x, y, m
        x_sum = 0.0
        y_sum = 0.0
        m_sum = 0.0

        size = len(lines)
        if size == 0:
            return 0, 0

        for line in lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m = m_sum / size
        b = y_avg - m * x_avg

        return m, b


    # get lpos, rpos
    def get_line_pos(self, img, lines, left=False, right=False):
        m, b = self.get_line_params(lines)

        if m == 0 and b == 0:
            if left:
                pos = 0
            if right:
                pos = self.Width
        else:
            y = self.Gap/2
            pos = (y - b) / m

            b += self.Offset
            x1 = (self.Height - b) / float(m)
            x2 = ((self.Height / 2) - b) / float(m)

            #cv2.line(img, (int(x1), self.Height), (int(x2), (self.Height / 2)), (255, 0, 0), 3)

        return img, int(pos), m


    # show image and return lpos, rpos
    def process_image(self, frame):
        cv2.imshow("frame", frame)
        frame_copy = copy.deepcopy(frame)
        # blur
        kernel_size = 5
        blur = cv2.GaussianBlur(frame_copy, (kernel_size, kernel_size), 0)

        #ROI set
        roi_img = blur[self.Offset : self.Offset+self.Gap, 0:640]

        #gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        
        # LAB divide
        L, _, _ = cv2.split(cv2.cvtColor(roi_img, cv2.COLOR_BGR2LAB))
        # cv2.imshow("L", L)
        _, lane = cv2.threshold(L, 140, 255, cv2.THRESH_BINARY)

        img = cv2.bitwise_not(lane)

        # cv2.imshow("img", img)

        # canny edge
        low_threshold = 40
        high_threshold = 70
        edge_img = cv2.Canny(np.uint8(img), low_threshold, high_threshold)

        #edge_img2 = cv2.Canny(np.uint8(gray), low_threshold, high_threshold)

        cv2.imshow("edge_img", edge_img)
        #cv2.imshow("edge_img2", edge_img2)

        all_lines = cv2.HoughLinesP(edge_img, 1, math.pi / 180, 45, 20, 5)

        # divide left, right lines
        if all_lines is None:
            return 0, 640
        st = time.time()
        left_lines, right_lines = self.divide_left_right(all_lines)
        
        # get center of lines
        frame_copy, lpos, m_left = self.get_line_pos(frame_copy, left_lines, left=True)
        frame_copy, rpos, m_right = self.get_line_pos(frame_copy, right_lines, right=True)
        #print(time.time()-st)
        # pos weighted_moving_average
        self.lpos_deque.append(lpos)
        if len(self.lpos_deque) == 10:
            lpos_avg = self.weighted_moving_average(8, self.lpos_deque)
            lpos = lpos_avg
            self.lpos_deque.popleft()
        else:
            lpos_avg = lpos

        self.rpos_deque.append(rpos)
        #print(lpos)
        if len(self.rpos_deque) == 10:
            rpos_avg = self.weighted_moving_average(8, self.rpos_deque)
            rpos = rpos_avg
            self.rpos_deque.popleft()
        else:
            rpos_avg = rpos
        

        # ROI
        # roi_left = edge_img[240 : 270, 200 : 240]
        # roi_right = edge_img[240 : 270, 400 : 440]


        # vertices1 = np.array([[(10, Gap), (125, 0), (160, 0), (180, Gap)]], dtype=np.int32)
        # vertices2 = np.array([[(Width - 190, Gap), (Width - 170, 0), (Width - 130, 0), (Width-10, Gap)]], dtype=np.int32)
        # roi = edge_img[Offset: Offset + Gap, 0: Width]
        # roi_frame1 = ROI(roi, [vertices1])
        # roi_frame2 = ROI(roi, [vertices2])
        # roi_frame = cv2.add(roi_frame1, roi_frame2)
        # roi = np.uint8(roi_frame)

        #cv2.imshow('roi', roi)
        #cv2.imshow('roi_left', roi_left)
        #cv2.imshow('roi_right', roi_right)
        # HoughLinesP
        # all_lines = cv2.HoughLinesP(roi, 1, math.pi / 180, 30, 30, 10)
        # center_left = cv2.HoughLinesP(roi_left, 1, math.pi / 180, 10, 10, 5)
        # center_right = cv2.HoughLinesP(roi_right, 1, math.pi / 180, 10, 10, 5)
        # center_left_1 = divide_left(center_left)
        # center_right_2 = divide_right(center_right)

        # draw lines
        frame_copy = self.draw_lines(frame_copy, left_lines)
        frame_copy = self.draw_lines(frame_copy, right_lines)
        frame_copy = cv2.line(frame_copy, (230, 235), (410, 235), (255, 255, 255), 2)

        # draw rectangle
        frame_copy = self.draw_rectangle(frame_copy, lpos, rpos)
        # roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        # roi2 = draw_rectangle(roi2, lpos, rpos)
        cv2.imshow("frame_copy", frame_copy)
        # show image
        # cv2.imshow('calibration', frame)

        return lpos, rpos

    def weighted_moving_average(self, n, pos_deque):
        weight, avg = 0, 0
        for i in range(1, n+1):
            weight += i*pos_deque[i-1]
            avg += i
        pos_avg = weight/avg
        return pos_avg

    def start(self):
        global pub
        global image
        global cap
        global Width, Height

        print
        "---------- Xycar A2 v1.0 ----------"
        rospy.sleep(0.5)

        while not rospy.is_shutdown():
            while not self.image.size == (640 * 480 * 3):
                continue
            lpos, rpos = self.process_image(self.image)

            

            # drive(pid_angle, speed)
            #cv2.imshow('x', image)
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

        rospy.spin()


if __name__ == '__main__':
    lane_detection = LaneDetection()
    lane_detection.start()
