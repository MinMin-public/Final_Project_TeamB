#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, math
import time, rospy
import numpy as np

from xycar_msgs.msg import xycar_motor
from xycar import Xycar

class T_parking():
    
    def __init__(self):
        self.arData = {"DX":0.0, "DY":0.0, "DZ":1.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.motor_msg = xycar_motor()
        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)
        self.teamB = Xycar()
        self.t_parking_end = True

    def get_distance(self):
        distance = math.sqrt(pow(self.arData["DX"],2) + pow(self.arData["DZ"],2))
        return distance
        
    def get_angle(self):
        angle = math.degrees(np.arctan(self.arData["DX"] / self.arData["DZ"])) * 1.5
        return angle


    def T_parking(self):
        print('------------------ T-parking -----------------')
        
        self.arData = self.teamB.arData
        self.roll, self.pitch, self.yaw = self.teamB.roll, self.teamB.pitch, self.teamB.yaw
        #angle = self.get_angle()
        distance = self.get_distance()
        self.time_count = 0
        if (round(self.yaw, 1) > 3.0 or round(self.yaw, 1) < -3.0) or (distance < 0.74):
            angle = round(self.yaw, 1) * 11.5
            speed = -23
            #angle = t_parking.get_angle()
            #teamB_motor.drive_o(angle, 15)
            return angle, speed
        else:
            angle, speed = 0, 0
            return angle, speed
        #time_count += 1
        #if time_count >= 100
        #    angle, speed = self.get_angle(), -30
        #    time_count = 0
        #    return angle, speed
        #if time_cou
    def T_parking_again(self):
        print('------------------ T-parking-again -----------------')
        #if (-10 < artag.get_angle() < 10) and (0.75 < artag.get_distance() < 0.85): #????
        #    angle = 0
        #    speed = 0
        #    self.drive(angle, speed)
        #else:
        for cnt in range(30):
            angle = artag.get_angle()
            print('front')
            print('angle : ', angle)
            self.drive(angle, 25)
            rospy.sleep(0.1)
        for cnt in range(20): #???? 1
            print('back')
            print('angle : ', angle)
            self.drive(-artag.get_angle(), -35)
            rospy.sleep(0.1)
        for cnt in range(10):
            print('back')
            print('angle : ', angle)
            self.drive(artag.get_angle(), -35)
            rospy.sleep(0.1)
        for cnt in range(5):
            print('stop')
            print('angle : ', angle)
            self.drive(0, 0)
            rospy.sleep(0.1)
        rospy.sleep(2.0)
        """
        if (round(yaw, 1) > 5.0 or round(yaw, 1) < -5.0): 
            for cnt in range(first_count):
                angle = round(yaw, 1) * -4.0
                self.drive(angle, -25)
                rospy.sleep(0.1)
            for cnt in range(last_count):
                angle = round(yaw, 1) * 4.0
                self.drive(angle, -25)
                rospy.sleep(0.1)
        else:
            angle = 0
            speed = 0
            self.drive(angle, speed)
        """
