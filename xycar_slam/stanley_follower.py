#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import pickle
from stanley import StanleyControl

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Imu,Image
import cv2
import time, collections
from cv_bridge import CvBridge



class StanleyController(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.yaw = 0.0
        self.yaw_previous = collections.deque([])
        self.v = 0
        self.imu_data = 0.12
        self.state = "path_planning"
        self.stop_line = False
        self.previous_time = time.time()

        with open(
                "/home/nvidia/xycar_ws/src/xycar_slam/reference_path.pkl",
                "rb") as f_1:
            self.path = pickle.load(f_1)
        
        # with open(
        #         "/home/nvidia/xycar_ws/src/xycar_slam/reference_path_2.pkl",
        #         "rb") as f_2:
        #     self.path_2 = pickle.load(f_2)

        self.rear_x_previous = self.path['x'][0]
        self.rear_y_previous = self.path['y'][0]

        self.ego_pose_sub = rospy.Subscriber("tracked_pose", PoseStamped, self.PoseCallBack)
        self.ego_imu_sub = rospy.Subscriber("imu", Imu, self.ImuCallBack)

    def ImuCallBack(self, msg):
        self.imu_data = msg.linear_acceleration.z

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

        self.v = float(dist) / tact_time
        self.previous_time = present_time
        self.rear_x_previous = self.rear_x
        self.rear_y_previous = self.rear_y
        

    def GetMsg(self):
        start_time = time.time()
        delta = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v,
                               self.path['x'], self.path['y'], self.path['yaw'],
                               0.3, 0.7)
        
        print("delta time:", time.time()-start_time)
        print("self.v:",self.v)
        
        delta = delta #*50/20
        if delta < -50:
            delta = -50
        elif delta > 50:
            delta = 50

        msg = xycar_motor()
        #if self.imu_data <= -1 and 1.9 < self.rear_y < 2.4:
        #    self.state = "slope"
        #elif self.imu_data <= -2.5:
        #    self.state = "path_planning"


        #if self.state == "slope" and 2.5 < self.rear_y < 2.8 and self.v < 0.009:
        #   self.state = "path_planning2"
        msg.speed = 15
        msg.angle = delta
        
        if len(self.yaw_previous) == 5:
            if abs(self.yaw) >= abs(sum(self.yaw_previous)/5) + 0.17: # 10도 = 0.17라디안
                msg.speed = 10
                
            else:
                msg.speed = 15
        self.yaw_previous.popleft()
        self.yaw_previous.append(self.yaw)


        #if self.state == "path_planning":
        #    msg.angle = delta
        #    msg.speed = 20
        #elif self.state == "path_planning2":
        #    msg.angle = delta
        #    msg.speed = 38
        #elif self.state == "slope":
        #    msg.angle = 0
        #    msg.speed = -100

        return msg

if __name__ == '__main__':
    rospy.init_node("stanley_follower_node")

    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    stanley = StanleyController()

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        x = stanley.rear_x
        y = stanley.rear_y
        msg = stanley.GetMsg()
        motor_pub.publish(msg)


        r.sleep()
