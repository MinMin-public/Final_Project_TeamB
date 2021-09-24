#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time, rospy

from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan

class rotary_lidar:       
    def __init__(self):
        self.rotary_flag = True
        self.right = 0

    def lidar_callback(self):
        if self.right < 1.5: #car is detected on 45 degree side
            self.rotary_flag = False
        else:
            self.rotary_flag = True
        return self.rotary_flag
        
    
