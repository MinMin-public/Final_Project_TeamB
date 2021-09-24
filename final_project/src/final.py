#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time, cv2
import numpy as np
from cv_bridge import CvBridge
from xycar import Xycar
from Control.motor import Motor
from Perception.traffic_sign import Traffic_Sign
from Perception.hough import Hough
from Perception.interrupt import Interrupt

rospy.init_node('final_project')

teamB = Xycar()
teamB_motor = Motor()
green_light = Traffic_Sign()
teamB_line = Hough() 
traffic_cnt = 0

while not rospy.is_shutdown():
    if teamB.img is None:
        continue

    # while teamB.imu is None:
        # time.sleep(0.03)

    # while teamB.lidar is None:
        # rospy.sleep(0.03)

    # while teamB.ultra is None:
        # rospy.sleep(0.03)


    # print("ready")
    if teamB.mode == 0:
        green_light.img = teamB.img
        start = green_light.detect_greenlight()
        if start:
            traffic_cnt += start
        if traffic_cnt == 5:
            print("go")
            #green_light.img = None
            # del green_light
            teamB.mode += 1
            # green_light = Traffic_Sign()
            
            # pos, img = teamB_line.process_image(teamB.img)
            # angle = teamB_motor.drive_angle(pos)
            # angle = np.clip(angle, -30, 30)
            # print(angle)
            # teamB_line.draw_steer(img, angle)
            # for i in range(3):
            #     teamB_motor.drive_go(angle, 15)
            #     time.sleep(3)
            
    elif teamB.mode == 1:
        pos, img = teamB_line.process_image(teamB.img)
        angle = teamB_motor.drive_angle(pos)
        angle = np.clip(angle, -50, 50)
        print(angle)
        teamB_motor.drive_go(-angle, 18)
        # # teamB_line.draw_steer(img, angle)
        # for i in range(3):
        #     teamB_motor.drive_go(angle, 15)
        #     time.sleep(3)
        # teamB_sensor = Interrupt(teamB.lidar, teamB.ultra)
        # if teamB_sensor.detect_car == 1:
        #     for i in range(5):
        #         angle = 20
        #         teamB_motor.drive_go(angle, 15)
        #         time.sleep(1)
        #     for i in range(10):
        #         pos, img = teamB_line.process_image(teamB.img)
        #         angle = teamB_motor.drive_angle(pos)
        #         angle = np.clip(angle, -30, 15)
        #         teamB_motor.drive_go(angle, 15)
        #         time.sleep(3)
        #     del teamB_sensor
        #     teamB.mode += 1

        # print(angle)
        # teamB_line.draw_steer(img, angle)

    time.sleep(0.03)
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
