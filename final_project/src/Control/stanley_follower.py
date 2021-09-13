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

        self.lidar = None
        self.imu_data = None
        self.state = "path_planning"
        self.stop_line = False
        self.min_index = 0
        self.previous_time = time.time()
        self.count = 0

        with open(
                "/home/nvidia/xycar_ws/src/final_project/path/reference_path_lidar.pkl",
                "rb") as f_1:
            self.path = pickle.load(f_1)
        
        # with opesrc(
        #         "../../path/reference_path_2.pkl",
        #         "rb") as f_2:
        #     self.path_2 = pickle.load(f_2)

        self.rear_x_previous = self.path['x'][0]
        self.rear_y_previous = self.path['y'][0]

        self.ego_pose_sub = rospy.Subscriber("tracked_pose", PoseStamped, self.PoseCallBack)
        # self.ego_imu_sub = rospy.Subscriber("imu", Imu, self.ImuCallBack)

    def ImuCallBack(self, imu):
        #self.imu_data = msg.linear_acceleration.x
        orientation_q = imu.orientation
        _, self.imu_data, _ = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

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
        
    def filtering(self, x):
        return x > 0.15 and x < 1.3

    def filter_car(self, x):
        return x > 0.0 and x <= 0.6

    def interrupt(self):        
        left = [0] * 8

        for j in range(8):
            index = j*14 + 7
            left_temp = list(filter(self.filtering, self.lidar[index : index + 14]))
            if len(left_temp) != 0:
                left[j] = round(sum(left_temp)/len(left_temp), 2)
        
        left = list(filter(self.filter_car, left))
        print(left)
        if len(left) > 0:
            return 1
        return 0

    def GetMsg(self):
        start_time = time.time()
        delta, self.min_index = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v,
                               self.path['x'][self.min_index:self.min_index+5000], self.path['y'][self.min_index:self.min_index+5000], self.path['yaw'][self.min_index:self.min_index+5000],
                               0.25, 0.25)
        
        #print("delta time:", time.time()-start_time)

        delta = delta *50/20

        msg = xycar_motor()

        # self.state = "path_planning"

        # if -5.6 < self.rear_y < -5.4 and -5.73 < self.rear_x < -5.6:
        #     self.state = "path_planning"
        
        # if -5.65 < self.rear_y < -5.59 and -2.05 < self.rear_x < -1.98:
        #     self.state = "path_planning"
        # elif self.imu_data > 0.2:
        #     self.state = "stop"
        # elif self.imu_data > 0.2:
        #     self.state = "stop"

        # if self.count == 0 and (-6.7 < self.rear_x < -6.3 and 0.0 < self.rear_y < 0.3):
        #     self.count += 1
        #     self.state = "stop"
            
        # if self.count > 0 and self.detect_car():
        #     self.state = "path_planning"
        
        #else:
        #    self.state = "path_planning"
        

        if self.state == "path_planning":
            msg.speed = 15
            msg.angle = delta
        elif self.state == "stop":
            msg.speed = 0
            msg.angle = 0

        return msg

# if __name__ == '__main__':
#     rospy.init_node("stanley_follower_node")

#     motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
#     stanley = StanleyController()

#     r = rospy.Rate(40)
#     while not rospy.is_shutdown():
#         x = stanley.rear_x
#         y = stanley.rear_y
#         msg = stanley.GetMsg()
#         start = time.time()
#         if msg.speed == 0:
#             while time.time()-start <= 1:
#                 continue
#                 # print("stop!!!")
#         motor_pub.publish(msg)


#         r.sleep()
