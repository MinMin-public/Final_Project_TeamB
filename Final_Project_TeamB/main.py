#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time
import cv2
from std_msgs.msg import String, Int32MultiArray
from xycar_msgs.msg import xycar_motor
from traffic_sign import Traffic_Sign
        
def drive(angle, speed):
    global pub
    msg = xycar_motor()
    msg.angle = angle
    msg.speed = speed
    pub.publish(msg)

rospy.init_node('test')
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
traffic = Traffic_Sign()

while not rospy.is_shutdown():
    
    while traffic.image is None:
        time.sleep(0.03)
    traffic.detect_circle()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


