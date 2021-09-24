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
from Perception.yolo import YOLO
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
    yolo = YOLO()
    start = 0

    angle = 0
    time_count = 0
    left = 0
    right = 0
    

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
        # stanley.state = "path_planning"

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
                teamB.mode = "interrupt"

        # interrupt
        if teamB.mode == "interrupt":
            if -4.9 < stanley.rear_x < -4.6 and 0.0 < stanley.rear_y < 0.2:
                print("stop!! before interrupt")
                stanley.state = "stop"
            if stanley.state == "stop" and interrupt.detect_car():
                print("detect")
                time_count += 1         
                if time_count >= 80:
                    stanley.state = "path_planning"
                    print("go!!!!!!!!!!!!!!!!")
                    teamB.mode = "tparking_start"
                    time_count = 0

        # intersection - stopline + traffic sign
        
        if teamB.mode == "intersection":
            if -14.65 < stanley.rear_x < -14.3 and -3.2 < stanley.rear_y < -2.8:  
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
                    # msg = stanley.GetMsg()
            
        # intersection - t parking
        # if teamB.mode == "tparking_start":
        #     if -14.65 < stanley.rear_x < -14.5 and -2.96 < stanley.rear_y < 2.92: 
        #         print("t_parking!!")
        #         teamB.mode = "tparking_end"
        #         # msg = stanley.GetMsg()

        # intersection - t parking 
        if teamB.mode == "tparking_start":
            if -17 < stanley.rear_x < -16.6 and 0.1 < stanley.rear_y < 0.15 and t_parking_flag:
                stanley.state = "parking"
                print("t_parking end!!!")
                teamB.mode = "tparking"
        if teamB.mode == "tparking":
            t_parking_flag = False
            msg.angle, msg.speed = t_parking.T_parking()
            if msg.angle == 0 and msg.speed == 0:
                teamB.mode = "path_change"  
                stanley.state = "path_planning" 

        # path change
        if teamB.mode == "path_change":
            if -17 < stanley.rear_x < -15 and -0.95 < stanley.rear_y < -0.85:
                stanley.state = "path_planning"
                # msg = stanley.GetMsg()
                stanley.path = stanley.path_2
                teamB.mode = "yolo_person"

        #stop to person
        if teamB.mode == "yolo_person":
            if -19 < stanley.rear_x < -17.88 and -1.54 < stanley.rear_y < -1.45: # person stop
                # start = time.time()
                # while (time.time() - start) <= 1:
                #     teamB_motor.drive_go(msg.angle, msg.speed)
                stanley.state = "stop"
            if stanley.state == "stop":
                time_count += 1
                if time_count >= 200:
                    stanley.state = "path_planning"
                    time_count = 0
                    teamB.mode = "yolo_cat"


        #stop to cat
        if teamB.mode == "yolo_cat":
            if -16.45 < stanley.rear_x < -16.3 and -2.9 < stanley.rear_y < -2.87: # person stop
                # start = time.time()
                # while (time.time() - start) <= 1:
                #     teamB_motor.drive_go(msg.angle, msg.speed)
                stanley.state = "stop"
            if stanley.state == "stop":
                time_count += 1
                print("count:", time_count)
                if time_count >= 200:
                    stanley.state = "path_planning"
                    time_count = 0
                    teamB.mode = "path_change3"
        
        # path change to path3
        if teamB.mode == "path_change3":
            if -5.5 < stanley.rear_x < -4 and -5.9 < stanley.rear_y < -4:
                # mode = "path_follow"
                # msg = stanley.GetMsg()
                stanley.path = stanley.path_3
                teamB.mode = "slope"

        # slope
        if teamB.mode == "slope":
            if -2.05 < stanley.rear_x < -1.98 and -5.65 < stanley.rear_y < -5.59:
                stanley.state = "path_planning"
            elif stanley.imu_data > 0.2:
                stanley.state = "stop"
                teamB.mode = "rotary"

        # rotary - stopline
        if teamB.mode == "rotary":
            #if -2.0 < stanley.rear_x < -1.94 and -5.65 < stanley.rear_y < -5.58 and rotary_flag:  
            stanley.state = "stop"
            rotary.right = teamB.right
            print('rotary.right :', rotary.right)
            rotary_flag = rotary.lidar_callback()
            #print('rotary_flag : ', rotary_flag)
            #print(rotary_flag)
            if rotary_flag == False:
                stanley.state = "path_planning"
                teamB.mode = "obstacle"
        #obstacle
        if teamB.mode == "obstacle" and stanley.rear_y > -4.2 :
            obs.lidar = teamB.lidar
            direction = obs.detect_obstacle()
            print(direction)
            time_count += 1
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
		
            if left+right == 2:
                teamB.mode == "parallel"
                stanley.state = "path_planning"

        # parallel parking
        if teamB.mode=="parallel":
            #아래 좌표 범위 수정 요망
            if -2.0 < stanley.rear_x < -1.94 and -5.63 < stanley.rear_y < -5.61 and stanley.state=="path_planning":
                parallel.state=="searching"

            if parallel.state=="searching":
                if parallel.parallel_start():
                    stanley.state="stop"
                    parallel.state="right"

            if parallel.state=="right":
                if parallel.parallel_right():
                    msg.angle=30
                    msg.speed=10

                else:
                    parallel.state="left"

            if parallel.state=="left":
                if parallel.parallel_left():
                    msg.angle=-30
                    msg.speed=10

                else:
                    parallel.state="back"

            if parallel.state=="back":
                if parallel.parallel_back():
                    msg.angle=0
                    msg.speed=-20

                else:
                    msg.angle=0
                    msg.speed=0
            

    
        if stanley.state == "path_planning" or stanley.state == "stop":
            msg = stanley.GetMsg()
        if -4.9 < stanley.rear_x and -0.1 < stanley.rear_y < 0.3 and stanley.state == "path_planning":
            msg.speed = 13
        elif stanley.imu_data < -0.98 and stanley.state == "path_planning":
            msg.speed = 13
        elif stanley.imu_data >= -0.98 and stanley.state == "path_planning":
            msg.speed = 10

        

        teamB_motor.drive_go(msg.angle, msg.speed)
        
        #print(time.time()-start_time)
        print(stanley.state)
        print("imu:",stanley.imu_data)
        r.sleep()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
