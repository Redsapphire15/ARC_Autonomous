#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from navigation.msg import gps_data
import math
import imutils 
import sys
from navigation.msg import gps_data
import math
import time
import cv2
import numpy as np
import imutils
from traversal.msg import WheelRpm
from traversal.srv import *
from std_msgs.msg import Bool
import pyrealsense2 as rs
import std_msgs.msg as std_msgs
from ultralytics import YOLO
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics.utils.plotting import Annotator
from collections import defaultdict


class GoToGoal:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.update_pose)
        rospy.Subscriber('/gps_coordinates', gps_data, self.gps_callback)
        self.wheelrpm_pub = rospy.Publisher('motion',WheelRpm,queue_size=10)
        self.rate = rospy.Rate(10)
        self.dummy = None
        self.angle_thresh = 4
        self.pose = Point()
        self.goal = Point()

#from gps data include coordinates here
        
        self.goal_lat = input()
        self.goal_long = input()

    # def update_pose(self, data):
    #     self.pose.x = round(data.pose.pose.position.x, 4)
    #     self.pose.y = round(data.pose.pose.position.y, 4)

    def distance_to_goal(self):
        a = math.pow(math.sin((self.goal_lat - self.current_latitude)/2),2) + math.cos(self.goal_lat)*math.cos(self.current_latitude)*math.pow(math.sin((self.goal_long - self.current_longitude)/2),2)
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        self.distance = 6370 * c


    def get_angle_to_goal(self):
        a = math.atan2(self.goal_long - self.current_longitude,self.goal_lat - self.current_latitude)
        self.angle_to_turn = self.z_angle - a
        self.initial_yaw = self.z_angle
        if self.angle_to_turn < 0:
            rotation = "left"
            self.dummy = -1
        if self.angle_to_turn > 0:
            rotation = "right"
            self.dummy = 1

    def move_to_goal(self):
        while not rospy.is_shutdown():
            #distance_to_goal = self.get_distance_to_goal()
            if self.distance < 0.1:
                rospy.loginfo("Goal reached!")
                break # haversine used to get the distance
            else:
                # angle_to_goal = self.get_angle_to_goal()
                if abs(self.angle_to_turn) > 2:
                    vel_msg = Twist()
                    vel_msg.angular.z = 0.3 * self.dummy
                    vel_msg.linear.x = 0.0
                    self.velocity_publisher.publish(vel_msg)
                else:
                    vel_msg = Twist()
                    vel_msg.linear.x = 0.5
                    vel_msg.angular.z = 0.0
                    self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

    def gps_callback(self,msg):
        if(msg.latitude and  msg.longitude):
            self.current_latitude=msg.latitude
            self.current_longitude=msg.longitude
            
            #file_object=open("coordinates.txt","a")
            #file_object.write(f"latitude :{msg.latitude}, longitude :{msg.longitude} \n")
            #file_object.close()
        
    
    def yaw_callback(self,msg):
        if (self.initial_drift_angle==0):   #only for 1st time
            self.initial_drift_angle=self.z_angle
        self.z_angle = msg.data
    
    def spin(self):
        self.main()
        rate.sleep()

    def rotate(self,dir):
        print("Rotation has been called")
        msg = WheelRpm()
        msg.vel = 127
        msg.hb=False
        diff  = self.z_angle - self.initial_yaw
        if (diff > 120):
            diff = diff - 360
        elif (diff < -120):
            diff = diff + 360
        print("diff=",diff)
        '''
        if (-60<self.z_angle-self.initial_yaw<60):
            error=30
        else:
            error = 90-abs(self.z_angle-self.initial_yaw)
        '''
        print("Rotation angle:",self.rotate_angle)
        print("Initial Yaw:", self.initial_yaw)
        print("Current z angle", self.z_angle)
        error = self.rotate_angle-abs(diff)
        print("error=", error)
        #if self.direction == -1:
         #   self.rotate_angle = self.rotate_angle +2
        if (abs(error)>=0.5*self.angle_thresh):
            msg.omega=127+(dir*40)
            msg.vel=102
            print("Calling Rotate, printing Z angle below")
            print(error)
            self.wheelrpm_pub.publish(msg)
        else:
            msg.omega=127
            msg.vel = 127
            self.wheelrpm_pub.publish(msg)
            # self.z_angle=0

            self.initial_drift_angle=self.z_angle
            print("****ROTATE DONE*****")
            #self.distance = 10.0
            self.start_time = time.time()-10
            self.turn=False
            self.direction = "Not Available"
            rospy.sleep(2)



if __name__ == '__main__':
    try:
        rate = rospy.Rate(10)
        auto = GoToGoal()
        auto.move_to_goal()
        auto.spin()
    except rospy.ROSInterruptException:
        pass

