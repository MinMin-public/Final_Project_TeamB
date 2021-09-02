#!/usr/bin/env python
# -*- coding: utf-8 -*-

#from xycar_msgs.msg import xycar_motor
import time, rospy


class YOLO:
    """
    def __init__(self, image, bridge, Width, Height):
        self.image = np.empty(shape=[0])
        self.bridge = CvBridge()
        self.Width = 640
        self.Height = 480

        self.steering_pub = None
        self.box_data = []
        self.motor_msg = xycar_motor()
        self.person = None
        #self.init_node()
        rospy.init_node('human_follow')
        self.steering_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    """
           
    #def init_node(self):
        #rospy.init_node('human_follow')
        #self.steering_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    """
    def drive(self, angle, speed):
        self.motor_msg = xycar_motor()
        self.motor_msg.header.stamp = rospy.Time.now()
        self.motor_msg.angle = angle
        self.motor_msg.speed = speed
        self.steering_pub.publish(motor_msg)
    """
        
        
    def image_detect(self, box_data):
        print('image_detect')
        person = True
        cat = True
        dog = True
        cow = True
        #while box_data is None:
            #rate.sleep()
        object1 = "person"
        object2 = "cat"
        object3 = "dog"
        object4 = "cow"
        boxes = box_data
        if boxes is not None:
            for i in range(len(boxes.bounding_boxes)):
                if (boxes.bounding_boxes[i].Class == object1 and person == True) and boxes.bounding_boxes[i].probability >= 0.35:
                    person = False
                    print('Class : ', boxes.bounding_boxes[i].Class)
                    return True
                elif (boxes.bounding_boxes[i].Class == object2 and cat == True) and boxes.bounding_boxes[i].probability >= 0.35:
                    cat = False
                    print('Class : ', boxes.bounding_boxes[i].Class)
                    return True
                elif (boxes.bounding_boxes[i].Class == object3 and dog == True) and boxes.bounding_boxes[i].probability >= 0.35:
                    dog = False
                    print('Class : ', boxes.bounding_boxes[i].Class)
                    return True
                elif (boxes.bounding_boxes[i].Class == object4 and cow == True) and boxes.bounding_boxes[i].probability >= 0.35:
                    cow = False
                    print('Class : ', boxes.bounding_boxes[i].Class)
                    return True
                else:
                    return False
        
    #rate = rospy.Rate(10)
        

