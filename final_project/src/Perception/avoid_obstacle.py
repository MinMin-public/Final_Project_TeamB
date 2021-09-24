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
        return x > 0.15 and x < 1.3

    def filtering_zero(self, x):
        return x > 0

    def filter_obstacle(self, x):
        return x > 0.0 and x < 0.9

    def detect_obstacle(self):        
        left = [0] * 5
        right = [0] * 5

        for j in range(5):
            index = j*14 + 7
            left_temp = list(filter(self.filtering, self.lidar[index : index + 14]))
            right_temp = list(filter(self.filtering, self.lidar[-(index + 14) : -index]))
                
            if len(left_temp) != 0:
                if 0.15 < round(sum(left_temp)/len(left_temp), 2) < 0.5:
                    left[j] =  round(sum(left_temp)/len(left_temp), 2)
            if len(right_temp) != 0:
                if 0.15 < round(sum(right_temp)/len(right_temp), 2) < 0.5:
                    right[j] = round(sum(right_temp)/len(right_temp), 2)

        left = list(filter(self.filtering_zero, left))
        right = list(filter(self.filtering_zero, right))
        print("left: ", left)
        print("right: ", right)

        if len(right) == 0:
            right = 0
        else:
            right = sum(right) / len(right)

        if len(left) == 0:
            left = 0
        else:
            left = sum(left) / len(left)

        print("left: ", left)
        print("right: ", right)

        if right > left:
            return "right"
        elif left > right:
            return "left"
        else:
            return "None"
            
    def __del__(self):
        print("checked car")
