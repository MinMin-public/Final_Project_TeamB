#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, math
import time, rospy
import numpy as np

from xycar_msgs.msg import xycar_motor

class T_parking():
    
    def __init__(self):
        self.arData = {"DX":0.0, "DY":0.0, "DZ":1.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
        self.roll, self.pitch, self.yaw = 0, 0, 0

        self.first_count = 27
        self.last_count = 10
        self.left_count = 100
        self.right_count = 25
        self.stop_count = 20

    def get_distance(self):
        distance = math.sqrt(pow(self.arData["DX"],2) + pow(self.arData["DZ"],2))
        return distance
        
    def get_angle(self):
        angle = math.degrees(np.arctan(self.arData["DX"] / self.arData["DZ"])) * 2.0
        return angle
        
    def get_yaw(self):
        return yaw

    def drive(self, angle, speed) :
        self.motor_msg.angle = angle
        self.motor_msg.speed = speed
        self.motor_pub.publish(self.motor_msg)

    def T_parking(self):
        print('------------------ T-parking -----------------')
        
        ######ar tag id? ?? ??
        #for cnt in range(self.left_count): 
        #    self.drive(-30, 20)
        #    rospy.sleep(0.1)
            
        for cnt in range(self.right_count): 
            self.drive(37, 10)
            rospy.sleep(0.1)

        for cnt in range(self.stop_count):
            self.drive(0, 0)
            rospy.sleep(0.1)
            
        """??? count? ?? ??
        for cnt in range(right_count): 
            self.drive(35, 10)
            rospy.sleep(0.1)
            
        for cnt in range(left_count): 
            self.drive(-30, 10)
            rospy.sleep(0.1)
            
        for cnt in range(right_count): 
            self.drive(35, 10)
            rospy.sleep(0.1)

        for cnt in range(stop_count):
            self.drive(0, 0)
            rospy.sleep(0.1)
        """
        """
        if distance < 0.4: #distance? ???? artag ?? ??
            for cnt in range(stop_count):
            self.drive(0, 0)
            rospy.sleep(0.1)
        """
        
        #print(angle)
        for cnt in range(self.first_count): #???? 1
            print('back')
            self.drive(-self.get_angle(), -30)
            rospy.sleep(0.1)
        #print(angle)
        for cnt in range(self.last_count):
            print('back')
            self.drive(self.get_angle()*0.5, -30)
            rospy.sleep(0.1)
        for cnt in range(self.stop_count):
            self.drive(0, 0)
            rospy.sleep(0.1)
        
        """
        #???? 2
        yaw = artag.get_yaw()
        angle = artag.get_angle()
        print('angle : ', angle)
        #while True:
            #yaw = artag.get_yaw()
            #print(yaw)
        if (round(yaw, 1) > 1.0 or round(yaw, 1) < -3.0): 
            for cnt in range(first_count):
                angle = round(yaw, 1) * 8.0
                self.drive(angle, -35)
                rospy.sleep(0.1)
            for cnt in range(last_count):
                angle = round(yaw, 1) * -8.0
                self.drive(angle, -35)
                rospy.sleep(0.1)
            
        else:
            angle = 0
            speed = 0
            self.drive(angle, speed)
        """

        
        return angle 
        
    def T_parking_end(self):
        #####t-parking-end
        print('------------------ T-parking-end -----------------')
        for i in range(10):
            self.drive(0, 25)
            rospy.sleep(0.1)
        for i in range(20):
            self.drive(-50, 25)
            rospy.sleep(0.1)
        return True #hough_flag
        
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

