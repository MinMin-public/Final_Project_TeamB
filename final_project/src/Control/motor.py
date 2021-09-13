#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time
from xycar_msgs.msg import xycar_motor
from pid import LateralPlant

class Motor():

    def __init__(self):

        self.motor_msg = xycar_motor()
        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)
        self.center_x = 320
        self.pid = LateralPlant(x=320.0, y=0.0, yaw=0.0, v=0.0)

        self.kp = 0.4
        self.kd = 0.0005
        self.ki = 0.03
        self.dt = 0.01

        self.int_error = 0.0
        self.prev_error = 0.0

    def drive_angle(self, pos):
        pos_center = (pos[0] + pos[1]) / 2
        self.prev_error = self.center_x - pos_center

        error = pos_center - self.center_x

        diff_error = error - self.prev_error
        self.prev_error = error
        self.int_error += error

        # steer = - self.kp * error - self.kd * diff_error/self.dt - self.ki * self.int_error
        steer = - self.kp * error
        print(steer)
        self.pid.update(steer)
        return steer

    def drive_go(self, angle, speed):
        self.motor_msg.angle = angle
        self.motor_msg.speed = speed
        self.motor_pub.publish(self.motor_msg)

