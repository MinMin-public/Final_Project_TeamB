#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math
import numpy as np
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan

class Detect_Obstacle():

    def __init__(self):
        # self.lidar = lidar
        self.lidar = None
        self.cnt = 0

    def filtering(self, x):
        return x > 0.15 and x < 1.0

    def filtering_zero(self, x):
        return x > 0

    def filter_obstacle(self, x):
        return x > 0.0 and x < 0.7

    def detect_obstacle(self):        
        left = [0] * 3
        right = [0] * 3

        for j in range(3):
            index = j*14 + 21
            left_temp = list(filter(self.filtering, self.lidar[index : index + 14]))
            right_temp = list(filter(self.filtering, self.lidar[-(index + 14) : -index]))
                
            if len(left_temp) != 0:
                if 0.15 < round(sum(left_temp)/len(left_temp), 2) < 0.7:
                    left[j] =  round(sum(left_temp)/len(left_temp), 2)
            if len(right_temp) != 0:
                if 0.15 < round(sum(right_temp)/len(right_temp), 2) < 0.7:
                    right[j] = round(sum(right_temp)/len(right_temp), 2)

        left = list(filter(self.filtering_zero, left))
        right = list(filter(self.filtering_zero, right))
        print("left: ", left)
        print("right: ", right)

        if len(right) > len(left):
            return "right"
        elif len(left) > len(right):
            return "left"
        else:
            return "None"
            
    def __del__(self):
        print("checked car")
