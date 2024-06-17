#!/usr/bin/env python3
import sys
import rospy
from navigation.msg import gps_data
from geometry_msgs.msg import Twist
import math
import time
import cv2
import numpy as np
import imutils
from traversal.msg import WheelRpm
from traversal.srv import *
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import pyrealsense2 as rs
import threading
import std_msgs.msg as std_msgs
from ultralytics import YOLO
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics.utils.plotting import Annotator
from collections import defaultdict
from collections import OrderedDict
import cv2.aruco as aruco
from cv2.aruco import Dictionary
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu



class Lidar_Detection():
    def __init__(self):
        rospy.Subscriber('/galileo/laser_scan', LaserScan, self.lidar_callback)
        self.pub = rospy.Publisher('/galileo/cmd_vel', Twist, queue_size=10)
        self.thresh = 0.5
        self.left_check = 10
        self.right_check = 10
        

    def lidar_callback(self, data):
        print("check1")
        self.left_check = data.ranges[20]
        print("left", self.left_check)
        self.right_check = data.ranges[699]
        print("right", self.right_check)
        # self.mid = (data[0] + data[719])/2

    def main(self):
        twist = Twist()
        twist.linear.x = 1
        if self.left_check <= 1:
            twist.angular.z = 0.5
        if self.right_check<=1:
            twist.angular.z = -0.5
        self.pub.publish(twist)

    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()

        else:
            print("hi")


if __name__=="__main__":
    rospy.init_node("lava", anonymous=True)
    rate = rospy.Rate(10)
    ahh = Lidar_Detection()
    ahh.spin()