#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy, math
import numpy as np
import tf
import pickle
from stanley import StanleyControl

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Imu,Image
from sensor_msgs.msg import LaserScan
import cv2
import time, collections
from cv_bridge import CvBridge



class StanleyController(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.yaw = 0.0
        self.v = 0
        self.imu_data = 0.0
        self.state = "stop"
        self.count = 0
        self.previous_time = time.time()
        self.k = 0.8

        self.lidar = None

        with open(
                "/home/nvidia/xycar_ws/src/final_project/path/reference_path_final1.pkl",
                "rb") as f_1:
            self.path_1 = pickle.load(f_1)
        
        with open(
                "/home/nvidia/xycar_ws/src/final_project/path/reference_path_final2.pkl",
                "rb") as f_2:
            self.path_2 = pickle.load(f_2)

        with open(
                "/home/nvidia/xycar_ws/src/final_project/path/reference_path_final3.pkl",
                "rb") as f_3:
            self.path_3 = pickle.load(f_3)
        self.rear_x_previous = self.path['x'][0]
        self.rear_y_previous = self.path['y'][0]

        self.ego_pose_sub = rospy.Subscriber("tracked_pose", PoseStamped, self.PoseCallBack)
        # self.ego_imu_sub = rospy.Subscriber("imu", Imu, self.ImuCallBack)

    def PoseCallBack(self, msg):
        #print("tact_time:", tact_time)
        self.rear_x = msg.pose.position.x
        self.rear_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        dx = self.rear_x - self.rear_x_previous
        dy = self.rear_y - self.rear_y_previous
        dist = np.hypot(dx, dy)

        present_time = time.time()
        tact_time = present_time - self.previous_time

        self.v = 0.6 #float(dist) / tact_time # 0.35
        self.previous_time = present_time
        self.rear_x_previous = self.rear_x
        # self.0.35 = self.rear_y
        

    def GetMsg(self):
        delta = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v,
                               self.path['x'], self.path['y'], self.path['yaw'], 0.2, self.k)

        delta = delta *50/20
        msg = xycar_motor()
        if self.state == "path_planning":
            msg.speed = 9
            msg.angle = delta + 2
        elif self.state == "stop":
            msg.speed = 0
            msg.angle = 0
        
        return msg

