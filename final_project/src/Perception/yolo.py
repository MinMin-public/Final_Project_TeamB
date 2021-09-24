#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time, rospy
from xycar import Xycar
from xycar_msgs.msg import xycar_motor

class YOLO:      
    def __init__(self): 
        self.Class = None
        self.msg = xycar_motor()
        self.person = True
        self.cat = True

    def msg_callback(self):
        if self.Class == 'person':
            self.person = False
            msg.angle = 0
            msg.speed = 0
            return msg
        elif self.Class == 'cat':
            self.cat = False
            msg.angle = 0
            msg.speed = 0
            return msg

        
        

