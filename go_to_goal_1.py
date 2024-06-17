#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from navigation.msg import gps_data
from tf.transformations import euler_from_quaternion
import math
import imutils 
import numpy as np
from sensor_msgs.msg import Imu
from traversal.msg import WheelRpm

class GoToGoal():
    def __init__(self):
        #rospy.init_node('go_to_goal', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/motion', WheelRpm, queue_size=10)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        #rospy.Subscriber('/gps_coordinates', gps_data, self.gps_callback)
        self.rate = rospy.Rate(10)
        self.dummy = 0
        self.odom_self = [0,0]
        self.initial_odom = [0,0]
        self.a = 0
        self.initial_yaw = 0
        self.current_pitch,self.current_roll, self.current_yaw=0,0,0
        self.current_latitude, self.current_longitude = 0,0
        self.angle_to_turn = 0
        self.gps_publisher = rospy.Publisher("gps_from_location1", gps_data, queue_size=10)


#from gps data include coordinates here
        self.goal_lat = float(input("enter goal lat"))
        self.goal_long = float(input("enter goal long"))


    # def update_pose(self, data):

    def distance_to_goal(self):
        a = math.pow(math.sin((self.goal_lat - self.current_latitude)/2),2) + math.cos(self.goal_lat)*math.cos(self.current_latitude)*math.pow(math.sin((self.goal_long - self.current_longitude)/2),2)
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        self.distance = 6370 * c
        #self.distance = math.sqrt((self.goal_lat - self.current_latitude)**2 +(self.goal_long - self.current_longitude)**2)
        
    def odom_callback(self, data):
        #print("Odometry callback called.")
        #self.current_latitude = msg.pose.pose.position.x
        #self.current_longitude = msg.pose.pose.position.y
        #self.current_pose = msg.pose.pose
        #rospy.loginfo("Current Pose: {}".format(self.current_pose))
        self.odom_self[0] = data.pose.pose.position.x
        self.odom_self[1] = -data.pose.pose.position.y
        if self.odom_initialized == False:
            self.initial_odom[0] = self.odom_self[0]
            self.initial_odom[1] = self.odom_self[1]
            self.odom_initialized = True
        self.odom_self[0] = self.odom_self[0] - self.initial_odom[0]
        self.odom_self[1] = self.odom_self[1] - self.initial_odom[1]
    
    def imu_callback(self, data):
        current_x = data.orientation.x
        current_y = data.orientation.y
        current_z = data.orientation.z
        current_w = data.orientation.w
        self.current_pitch, self.current_roll, self.current_yaw = euler_from_quaternion(
            current_x, current_y, current_z, current_w)

        if self.initial_yaw == 0:
            # self.current_pitch = self.current_pitch*180/3.14 - self.initial_pitch
            # self.current_roll = self.current_roll*180/3.14 - self.initial_roll
            self.initial_yaw = self.current_yaw#*180/math.pi
        self.current_yaw = -self.current_yaw + self.initial_yaw
        if self.current_yaw < -math.pi*2/3:
            self.current_yaw = self.current_yaw + math.pi*2
        elif self.current_yaw > math.pi*2/3:
            self.current_yaw = self.current_yaw - math.pi*2


    def get_angle_to_goal(self):
        # if hasattr(self, 'current_pose'):  # Check if current_pose attribute exists
        a = math.atan2(self.goal_long - self.current_longitude, self.goal_lat - self.current_latitude)
        # quaternion = (
        #     self.current_pose.orientation.x,
        #     self.current_pose.orientation.y,
        #     self.current_pose.orientation.z,
        #     self.current_pose.orientation.w
        # )
        # euler = euler_from_quaternion(quaternion)
        # self.z_angle = euler[2]
        self.angle_to_turn = self.current_yaw - a
        print(self.angle_to_turn)
        if self.angle_to_turn < 0:
            rotation = "left"
            self.dummy = -1
        elif self.angle_to_turn > 0:
            rotation = "right"
            self.dummy = 1
        print("rotating", rotation)
        #else:
         #   rospy.logwarn("No 'current_pose' attribute. Skipping angle calculation.")
    def move_to_goal(self):
        print("distance to cover",self.distance)
        distance_travelled = np.sqrt((self.odom_self[1]-self.initial_odom[1])**2 + (self.odom_self[0]-self.initial_odom[0]**2))
        print("distance travelled", distance_travelled)
        if abs(distance_travelled -self.distance) < 0.3:
            rospy.loginfo("Goal reached!")
            vel_msg = WheelRpm()
            vel_msg.vel = 0
            vel_msg.omega = 0
            self.velocity_publisher.publish(vel_msg)
            gps_msg = gps_data()
            gps_msg.latitude = self.current_latitude
            gps_msg.longitude = self.current_longitude
            self.gps_publisher.publish(gps_msg)
             # haversine used to get the distance
        else:
            if (self.current_yaw - self.angle_to_turn) > 0.105:
                vel_msg = WheelRpm()
                vel_msg.omega = 20 * self.dummy*(-1)
                vel_msg.vel = 0
                self.velocity_publisher.publish(vel_msg)
            elif(self.current_yaw - self.angle_to_turn) < -0.105:
                vel_msg = WheelRpm()
                vel_msg.omega = 20* self.dummy
                vel_msg.vel= 0
                self.velocity_publisher.publish(vel_msg)
            else:
                vel_msg = WheelRpm()
                vel_msg.vel = 20
                vel_msg.omega= 0
                self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

    def gps_callback(self,msg):
        if(msg.latitude and  msg.longitude) and self.a == 0:
            self.current_latitude=msg.latitude
            self.current_longitude=msg.longitude
            self.a += 1
            #file_object=open("coordinates.txt","a")
            #file_object.write(f"latitude :{msg.latitude}, longitude :{msg.longitude} \n")
            #file_object.close()
        
    
    def yaw_callback(self,msg):
        if (self.initial_drift_angle==0):   #only for 1st time
            self.initial_drift_angle=self.z_angle
        self.z_angle = msg.data

    # def odom_callback(self,msg):
    #     self.current_latitude = msg.pose.pose.position.x
    #     self.current_longitude = msg.pose.pose.position.y
    #     self.odometry_data = msg

    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            self.rate.sleep()

    def main(self):
        #self.goal_lat = float(input('enter x:'))
        #self.goal_long = float(input('enter y:'))
        # print(self.current_pose)
        #rospy.loginfo("Current Pose: {}".format(self.current_pose))
        self.distance_to_goal()
        self.get_angle_to_goal()
        self.move_to_goal()


if __name__ == '__main__':
    try:
        rospy.init_node('go_to_goal', anonymous=True)
        rate = rospy.Rate(10)
        auto = GoToGoal()
        auto.spin()
    except rospy.ROSInterruptException:
        pass

