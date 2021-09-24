#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan

class Interrupt():

    def __init__(self):
        # self.lidar = lidar
        self.lidar = None
        self.cnt = 0

    def filtering(self, x):
        return x > 0.15 and x < 1.0

    def filter_car(self, x):
        return x > 0.0 and x < 0.8

    def detect_car(self):        
        left = [0] * 8

        for j in range(8):
            index = j*14 + 7
            left_temp = list(filter(self.filtering, self.lidar[index : index + 14]))
                
            if len(left_temp) != 0:
                left[j] = round(sum(left_temp)/len(left_temp), 2)
        
        left = list(filter(self.filter_car, left))
        if len(left) > 0:
            return True
            
    def __del__(self):
        print("checked car")
