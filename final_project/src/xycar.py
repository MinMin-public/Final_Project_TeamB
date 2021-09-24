#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, time, math

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, Imu
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge
import tf

class Xycar():

    def __init__(self):
        self.mode = "start"

        self.img = None
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

        self.Class = None

        self.imu = None
        self.imu_sub = rospy.Subscriber("imu", Imu, self.imu_callback)

        self.lidar = None
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.right = 0
        self.left = 0

        self.ultra = [0,0,0,0,0]
        self.ultra_sub = rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, self.ultra_callback)

        self.arData = {"DX":0.0, "DY":0.0, "DZ":1.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
        self.id = 0
        self.roll, self.pitch, self.yaw = 0, 0, 0
        self.artag_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback)

        #self.Class = None
        #self.yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, yolo_callback)

    def img_callback(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    
    def imu_callback(self, data):
        orientation_q = data.orientation
        _, self.imu, _ = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

    def lidar_callback(self, data):
        self.lidar = data.ranges
        fr_list = self.lidar[-70:-20] #[int(315 * increment)]
        fr_list = list(fr_list)
        fr_list[:] = [value for value in fr_list if value != 0]
        self.right = (sum(fr_list) / 50)
        #fl_list = self.lidar[25:75] #[int(45 * increment)]
        #fl_list = list(fl_list)
        #fl_list[:] = [value for value in fl_list if value != 0]
        #self.left = (sum(fl_list) / 50)
        
        
        
    def ultra_callback(self, data):
        #print(data.data)
        self.ultra[0] = data.data[0]
        self.ultra[1] = data.data[4]
        self.ultra[2] = data.data[5]
        self.ultra[3] = data.data[6]
        self.ultra[4] = data.data[7]

    def ar_callback(self, data):
        for i in data.markers :
            self.id = i.id
            self.arData["DX"] = i.pose.pose.position.x
            self.arData["DY"] = i.pose.pose.position.y
            self.arData["DZ"] = i.pose.pose.position.z

            self.arData["AX"] = i.pose.pose.orientation.x
            self.arData["AY"] = i.pose.pose.orientation.y
            self.arData["AZ"] = i.pose.pose.orientation.z
            self.arData["AW"] = i.pose.pose.orientation.w
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion((self.arData["AX"], self.arData["AY"], self.arData["AZ"], self.arData["AW"]))
        self.roll = math.degrees(self.roll)
        self.pitch = math.degrees(self.pitch) #math.degrees(yaw)
        self.yaw = math.degrees(self.yaw)    #math.degrees(pitch)
            

    def yolo_callback(self, data):
        boxes = data

        if boxes is not None:
            for i in range(len(boxes.bounding_boxes)):
                self.Class = boxes.bounding_boxes[i].Class
    

