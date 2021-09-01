import rospy, time

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Xycar():

    def __init__(self):
        self.mode = 1

        self.img = None
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

        # self.imu = None
        # self.imu_sub = rospy.Subscriber("imu", Imu, imu_callback)

        # self.lidar = None
        # self.lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

        # self.ultra = None
        # self.ultra_sub = rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback)

        # self.motor_msg = xycar_motor()
        # self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)

    def img_callback(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    
    # def imu_callback(self, data):
    #     self.imu = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

    # def lidar_callback(self, data):
    #     self.lidar = data.ranges

    # def ultra_callback(self, data):
    #     ultra = data.data

