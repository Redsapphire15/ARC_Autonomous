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
        self.pub = rospy.Publisher('/revised_scan', Float32MultiArray, queue_size = 10)
        self.diff_theta = rospy.Publisher('/diff_theta', Float32, queue_size = 10)
        self.vel_pub = rospy.Publisher('/galileo/cmd_vel', Twist, queue_size=10)
        self.optimal_angle = None
        self.rev_scan = Float32MultiArray()
        rospy.Subscriber('/galileo/laser_scan', LaserScan, self.lidar_callback)
        # rospy.Subscriber('/galileo/imu', Imu, self.imu_callback)
        self.yaw_angle = 0

    def clustermax(self, array):
        # Checking for > 1 m
        array = [min(x, 1) for x in array]

        # To find the largest cluster of 1's in the array
        max_cluster_length = 0
        max_cluster_start = -1
        current_cluster_length = 0
        current_cluster_start = -1
        print("check2")

        for i, value in enumerate(array):
            if value == 1:
                if current_cluster_length == 0:
                    current_cluster_start = i
                current_cluster_length += 1
            else:
                if current_cluster_length > max_cluster_length:
                    max_cluster_length = current_cluster_length
                    max_cluster_start = current_cluster_start
                current_cluster_length = 0

        # Check if the last cluster is the largest
        if current_cluster_length > max_cluster_length:
            max_cluster_length = current_cluster_length
            max_cluster_start = current_cluster_start

        return max_cluster_start, max_cluster_length


    # def imu_callback(self, data):
        # self.yaw_angle = data.z_angle

    def lidar_callback(self,lidar_data):
        # 0 corresponds to the point opposite to the cable of the LIDAR
        # from that, angle increments anticlockwise
        # each step is 360 / 1153 = 0.31222896791 degrees
        print("check1")
        left = lidar_data.ranges[230 : 231]
        right = lidar_data.ranges[923 : 924]

        if(left < 10 and right < 10):
            diff = right - left
            if(diff > 1):
                print("WARNING : MOVE  RIGHT")
            elif(diff < -1):
                print("WARNING : MOVE LEFT")

        req_range = list(lidar_data.ranges[0 : 144])
        req_range.reverse()

        temp_append = list(lidar_data.ranges[1009 : 1153])
        temp_append.reverse()

        req_range.extend(temp_append)

        

        # 0 - 143 : left part (upto -45 degrees) (143 is center)
        # 144 - 287 : right part (upto +45 degrees)

        # Clustering
        max_cluster_start, max_cluster_length = self.clustermax(req_range)
        optimal_step = max_cluster_start + (max_cluster_length / 2)

        #optimal angle is +ve for turning clockwise and -ve for anti-clockwise
        self.optimal_angle = (optimal_step * (90 / 287)) - 45
        print("optimal angle:", self.optimal_angle)


    def main(self):
        twist = Twist()
        initial_yaw = 0
        twist.linear.x = 0.4
        print("self.optimal_angle:", self.optimal_angle)
        if self.optimal_angle != 0 and self.optimal_angle is not None:
            #initial_yaw = self.yaw_angle
            twist.angular.z = 0.5* self.optimal_angle
        self.vel_pub.publish(twist)
    
    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()

        else:
            print("Waiting")

if __name__ == "__main__":
    rospy.init_node("lava", anonymous=True)
    rate = rospy.Rate(10)
    ahh = Lidar_Detection()
    ahh.spin()
    
        


        





