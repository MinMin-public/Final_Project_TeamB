#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time, cv2
import numpy as np
from xycar import Xycar
from Control.motor import Motor
from Perception.traffic_sign import Traffic_Sign
from Perception.hough import Hough

rospy.init_node('final_project')
teamB = Xycar()
teamB_motor = Motor()
teamB_line = Hough()
traffic_cnt = 0

while not rospy.is_shutdown():
    while teamB.img is None:
        time.sleep(0.03)

    # while teamB.imu is None:
    #     time.sleep(0.03)

    # while teamB.lidar is None:
    #     time.sleep(0.03)

    # while teamB.ultra is None:
    #     time.sleep(0.03)

    # if teamB.mode == 0:
    #     green_light = Traffic_Sign(teamB.img)
    #     # green_light.detect_greenlight()
    #     if green_light.detect_greenlight():
    #         traffic_cnt += green_light.detect_greenlight()
    #     if traffic_cnt == 5:
    #         del green_light
    #         pos, img = teamB_line.process_image(teamB.img)
    #         angle = teamB_motor.drive_angle(pos)
    #         angle = np.clip(angle, -30, 30)
    #         print(angle)
    #         teamB_line.draw_steer(img, angle)
    #         # teamB_motor.drive_go(angle, 20)
    #         time.sleep(3)
    #         teamB.mode += 1
    if teamB.mode == 1:
        pos, img = teamB_line.process_image(teamB.img)
        angle = teamB_motor.drive_angle(pos)
        angle = np.clip(angle, -30, 30)
        print(angle)
        teamB_line.draw_steer(img, angle)

    time.sleep(0.03)
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

