#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time, cv2
import numpy as np
from xycar import Xycar
from Control.motor import Motor
from Control.stanley_follower import StanleyController
from Perception.traffic_sign import Traffic_Sign
from Perception.hough_bev import Hough
from Perception.interrupt import Interrupt
from Perception.stopline import Stopline
from Perception.avoid_obstacle import Detect_Obstacle
#from Perception.yolo import YOLO
from Perception.rotary_lidar import rotary_lidar
from Control.T_Parking import T_parking
from Control.parallel import Parallel


if __name__ == '__main__':
    rospy.init_node('final_project')

    teamB = Xycar()
    teamB_motor = Motor()
    green_light = Traffic_Sign()
    stanley = StanleyController()
    interrupt = Interrupt() 
    obs = Detect_Obstacle()
    t_parking = T_parking()
    rotary = rotary_lidar()
    parallel = Parallel()

    line_interrupt = False
    detect_flag = True
    t_parking_flag = True
    rotary_flag = True
    path_change_flag = False
    start = 0
    yolo_person = 0
    yolo_cat = 0
    stanley.v = 1.3
    stanley.k = 0.8
    time_count = 0
    time_count_1 = 0
    left = 0
    right = 0
    stanley.state = "path_planning"

    r = rospy.Rate(40)
    while not rospy.is_shutdown():

        if teamB.img is None:
            continue

        if teamB.ultra is None:
            continue

        if teamB.imu is None:
            continue

        if teamB.lidar is None:
            continue

        start_time = time.time()
        stanley.lidar = teamB.lidar
        interrupt.lidar = teamB.lidar
        stanley.imu_data = teamB.imu

        print("mode: ", teamB.mode)
        # start - traffic sign
        if teamB.mode == "start":
            green_light.img = teamB.img
            start = green_light.detect_greenlight()
            if start:
                green_light.traffic_start_count += start
            if green_light.traffic_start_count == 5:
                print("go!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                green_light.img = None
                green_light.traffic_start_count = 0
                green_light.traffic_flag += 1
                start = 0
                stanley.state = "path_planning"
                teamB.mode = "intersection"

        # interrupt
        # if teamB.mode == "interrupt":
        #     if -3.8 < stanley.rear_x < -3.4 and -0.4 < stanley.rear_y < 0.0:
        #         detect_flag = 0
        #         print("stop!! before interrupt")
        #         stanley.state = "stop"
        #         if interrupt.detect_car():
        #             print("detect")
        #             detect_flag = 1         
        #         if detect_flag == 1:
        #             time_count += 1
        #             if time_count >= 10:
        #                 stanley.state = "path_planning"
        #                 print("go!!!!!!!!!!!!!!!!")
        #                 teamB.mode = "intersection"
        #                 time_count = 0

        # intersection - stopline + traffic sign
        if teamB.mode == "intersection":
            if -14.7 < stanley.rear_x < -14 and -4.2 < stanley.rear_y < -3.99:  
                stanley.state = "stop"
                green_light.img = teamB.img
                start = green_light.detect_greenlight()
                print(start)
                if start:
                    green_light.traffic_start_count += start
                if green_light.traffic_start_count == 5:
                    green_light.img = None
                    green_light.traffic_start_count = 0
                    stanley.state = "path_planning"
                    teamB.mode = "tparking_start"

        # intersection - t parking 
        if teamB.mode == "tparking_start":
            if -16.6 < stanley.rear_x < -16.4 and -1.4 < stanley.rear_y < -1.1 and t_parking_flag:
                stanley.state = "parking"
                print("t_parking end!!!")
                teamB.mode = "tparking"

        if teamB.mode == "tparking":
            t_parking_flag = False
            msg.angle, msg.speed = t_parking.T_parking()
            if msg.angle == 0 and msg.speed == 0:
                teamB.mode = "path_change"  
                stanley.state = "stop" 

        # path change
        if teamB.mode == "path_change":
            if -17 < stanley.rear_x < -15.5 and -2.4 < stanley.rear_y < -2.18:
                
                stanley.path = stanley.path_2
                path_change_flag = 1
            if path_change_flag == 1:
                time_count += 1
                if time_count >= 40:
                    time_count = 0
                    stanley.state = "path_planning"
                    teamB.mode = "yolo_person"

        #stop to person
        if teamB.mode == "yolo_person":
            if yolo_person == 0 and -17.85 < stanley.rear_x < -17.2 and -3.25 < stanley.rear_y < -3.13: # person stop
                yolo_person = 1
                stanley.state = "stop"
                
            if yolo_person == 1:
                time_count += 1
                if time_count >= 200:
                    stanley.state = "path_planning"
                    time_count = 0
                    teamB.mode = "yolo_cat"


        #stop to cat
        if teamB.mode == "yolo_cat":
            if yolo_cat == 0 and -16.04 < stanley.rear_x < -15.6 and -4.4 < stanley.rear_y < -4.2: # person stop
                yolo_cat = 1
                stanley.state = "stop"

            if yolo_cat == 1:
                time_count += 1
                if time_count >= 200:
                    stanley.state = "path_planning"
                    time_count = 0
                    teamB.mode = "slope"

        # slope
        if teamB.mode == "slope":
            if stanley.imu_data > 0.2:
                stanley.state = "stop"
                teamB.mode = "rotary"
                stanley.path = stanley.path_3

        # rotary - stopline
        if teamB.mode == "rotary":
            # if -1.47 < stanley.rear_x < -1.22 and -5.9 < stanley.rear_y < -5.7:
            #     stanley.state = "path_planning"
            #if -2.0 < stanley.rear_x < -1.94 and -5.65 < stanley.rear_y < -5.58 and rotary_flag:  
            # stanley.state = "stop"
            rotary.right = teamB.right
            print('rotary.right :', rotary.right)
            rotary_flag = rotary.lidar_callback()
            #print('rotary_flag : ', rotary_flag)
            #print(rotary_flag)
            time_count += 1
            if rotary_flag == False and time_count >= 100:
                time_count = 0
                stanley.state = "path_planning"
                teamB.mode = "obstacle"

        #obstacle
        if teamB.mode == "obstacle" and stanley.rear_y > -4.3 :
            obs.lidar = teamB.lidar
            direction = obs.detect_obstacle()
            print(direction)
            time_count += 1
            time_count_1 += 1
            if direction == "left":
                stanley.state = "left"
                msg.angle = 30
                msg.speed = 10
                left = 1
            elif direction == "right":
                stanley.state = "right"
                msg.angle = -30
                msg.speed = 10
                right = 1
            else:
                if time_count >= 40:
                    stanley.state = "path_planning"
                    time_count = 0
		
            if -0.4 > stanley.rear_y > -0.6:
                teamB.mode = "parallel"
                stanley.state = "path_planning"

        # parallel parking
        if teamB.mode=="parallel":
            #아래 좌표 범위 수정 요망
            if -0.29 < stanley.rear_x < -0.89 and -2 < stanley.rear_y < 2 and stanley.state=="path_planning":
                parallel.state="searching"

            if parallel.state=="searching":
                if parallel.parallel_start():
                    stanley.state="parallel"
                    parallel.state="right"

            if parallel.state=="right":
                if parallel.parallel_right():
                    msg.angle=30
                    msg.speed=12

                else:
                    parallel.state="left"

            if parallel.state=="left":
                if parallel.parallel_left():
                    msg.angle=-30
                    msg.speed=12

                else:
                    parallel.state="back"

            if parallel.state=="back":
                if parallel.parallel_back():
                    msg.angle=0
                    msg.speed=-21

                else:
                    msg.angle=0
                    msg.speed=0
            

    
        if stanley.state == "path_planning" or stanley.state == "stop":
            msg = stanley.GetMsg()
        if -4.9 < stanley.rear_x and -0.1 < stanley.rear_y < 0.3 and stanley.state == "path_planning":
            msg.speed = 15
            stanley.v = 1.3
            stanley.k = 0.8
        elif stanley.imu_data < -0.2 and stanley.state == "path_planning":
            msg.speed = 16
            msg.angle = 0
        # elif teamB.mode=="rotary":
        #     msg.speed = 11
        elif stanley.imu_data >= -0.4 and stanley.state == "path_planning":
            msg.speed = 10
            stanley.v = 0.6
            stanley.k = 0.8
        

        
        # if stanley.state == "right" or stanley.state == "left":

        teamB_motor.drive_go(msg.angle, msg.speed)
        
        print("ct",time.time()-start_time)
        print(stanley.state)
        #print("imu:",stanley.imu_data)
        r.sleep()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
