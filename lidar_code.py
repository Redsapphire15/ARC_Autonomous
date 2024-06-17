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
from nav_msgs.msg import Odometry
from scipy.interpolate import CubicSpline
from scipy.integrate import quad





class Lidar_Detection():
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.pub = rospy.Publisher('/motion', WheelRpm, queue_size=10)
        rospy.Subscriber('/odometry/filtered',Odometry, self.odom_callback)
        self.thresh = 0.5
        self.left_check = 10
        self.right_check = 10
        self.midpoint_list = []
        self.odom_self = [0,0]
        self.initial_odom = [0,0]
        self.memory_odom = [0,0]
        self.total_distance = 0
        self.midpoint_array = np.array((0))
        self.counter = 0
        self.distance = 0

    def lidar_callback(self, data):
        left_num = 5
        right_num = 685
        left_check_sum = 0
        right_check_sum = 0
        print("check1")
        for i in range(30):
            left_check_sum = data.ranges[left_num+i] + left_check_sum
            right_check_sum = data.ranges[right_num+i] + right_check_sum
        self.left_check = left_check_sum/30
        print("left", self.left_check)
        self.right_check = right_check_sum/30
        print("right", self.right_check)
    
    def odom_callback(self, data):
        self.memory_odom = self.odom_self
        self.odom_self[0] = data.pose.pose.position.x
        self.odom_self[1] = -data.pose.pose.position.y
        if self.odom_initialized == False:
            self.initial_odom[0] = self.odom_self[0]
            self.initial_odom[1] = self.odom_self[1]
            self.odom_initialized = True
        self.odom_self[0] = self.odom_self[0] - self.initial_odom[0]
        self.odom_self[1] = self.odom_self[1] - self.initial_odom[1]

    def main(self):
        twist = WheelRpm()
        twist.vel = 1
        if self.left_check <= 1:
            twist.omega = 0.5
        if self.right_check<=1:
            twist.angular.z = -0.5
        self.pub.publish(twist)
        self.counter +=1
        # odom_1 = np.asanyarray(self.odom_self)
        midpoint_tuple = self.odom_self 
        #midpoint_array = np.asanyarray(midpoint_tuple)
        self.midpoint_list.append(midpoint_tuple)
        self.midpoint_array = np.asanyarray(self.midpoint_list)
        print("I came inside main")
        self.distance+= np.sqrt((self.odom_self[1]-self.memory_odom[1])**2 + (self.odom_self[0]-self.memory_odom[0])**2)
        print("distance", self.distance)
        #print(self.midpoint_array[:,0])
        #x = self.midpoint_array[:,0]
        #y = self.midpoint_array[:,1]
        #if self.counter>=10:
         #   cs = CubicSpline(x,y)
           # x_new = np.linspace(x.min(), x.max(),100)
          #  y_new = cs(x_new)
            #cs_derivative = cs.derivative()
            #self.total_distance, _ = quad(np.sqrt(1+(cs_derivative(x))**2), x.min(),x.max())
            #print("total distance travelled", self.total_distance)


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