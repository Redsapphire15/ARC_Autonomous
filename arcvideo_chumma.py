#!/usr/bin/env python3
import sys
import rospy
#import rosbag
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
import threading
import std_msgs.msg as std_msgs
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics.utils.plotting import Annotator
from collections import defaultdict
from collections import OrderedDict
import cv2.aruco as aruco
from cv2.aruco import Dictionary


class Aruco_detect():
    def __init__(self):

        self.pipeline = rs.pipeline()
        config = rs.config()
        # rospy.init_node("arrowdetectmorethan3")
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        print("1")
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.turn = False
        self.z_angle = self.x_angle = self.y_angle = 0
        self.distance = 10.0 #set the initial distance as chumma random int
        self.ret = False
        self.initial_yaw = 0.0
        self.where_am_i = 100
        self.enc_data = 0
        self.start_angle = 55
        self.start_angle_1 = 90
        self.start_time = time.time()
        self.angles_dict = defaultdict(list)
        self.tmp_dict = defaultdict(list) # This is going to be my local memory variable of the last time two separate arucos 
        self.dire_attempt = 0
        self.state = True
        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10)

        try:
            rospy.Subscriber('state', Bool, self.state_callback)
            print("1")
            rospy.Subscriber('chatter',std_msgs.Float32,self.yaw_callback)
            print("2")
            rospy.Subscriber('enc_auto',std_msgs.Int8,self.enc_callback)
            print("3")  
            rospy.Subscriber('gps_coordinates', gps_data, self.gps_callback)
            print("4")
        except KeyboardInterrupt:
            # quit
            sys.exit()

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        align=rs.align(rs.stream.color)
        frames=align.process(frames)
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        cv2.imshow("Aruco Marker Detection",color_image)
        if not depth_frame or not color_frame:
            return False, None, None, None
        return True, depth_image, color_image, depth_frame

    def aruco_recognition(self,color_image,depth_image):
        # define an empty custom dictionary with
        self.aruco_dict = aruco.Dictionary(0, 5, 1)
        # add empty bytesList array to fill with 3 markers later
        self.aruco_dict.bytesList = np.empty(shape = (10, 4, 4), dtype = np.uint8)
        # add new marker(s)
        mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,0,0,1],[0,1,0,0,1],[1,0,0,0,0]], dtype = np.uint8)
        self.aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0],[0,1,0,0,1],[0,1,0,0,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,0,0,0],[1,0,1,1,1],[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0]], dtype = np.uint8)
        self.aruco_dict.bytesList[2] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,0,0,0],[1,0,1,1,1],[1,0,0,0,0],[0,1,1,1,0],[0,1,0,0,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[3] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,0,0,1],[0,1,1,1,0],[1,0,0,0,0]], dtype = np.uint8)
        self.aruco_dict.bytesList[4] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,1,1,1],[1,0,0,0,0],[0,1,0,0,1],[0,1,0,0,1],[1,0,1,1,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[5] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,1,1,1],[1,0,1,1,1],[1,0,1,1,1],[0,1,0,0,1],[1,0,0,0,0]], dtype = np.uint8)
        self.aruco_dict.bytesList[6] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,0,0,0],[0,1,0,0,1],[1,0,1,1,1],[1,0,1,1,1],[0,1,0,0,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[7] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,1,1,1],[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0],[1,0,1,1,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[8] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,1,1,1],[0,1,1,1,0],[1,0,0,0,0],[0,1,1,1,0],[1,0,0,0,0]], dtype = np.uint8)
        self.aruco_dict.bytesList[9] = aruco.Dictionary_getByteListFromBits(mybits)
        # save marker images
        for i in range(len(self.aruco_dict.bytesList)):
            cv2.imwrite("custom_aruco_" + str(i) + ".png", aruco.generateImageMarker(self.aruco_dict, i, 128))
        intrinsic_camera=np.array(((607.7380981445312, 0.0, 325.2829284667969),(0.0, 606.8139038085938, 238.5009307861328),(0,0,1.0)))
        distortion=np.array((0.0, 0.0, 0.0, 0.0, 0.0))
        while color_image!=None:
            img = np.asanyarray(color_image.get_data())
            print(img.shape)
            ret, output, ids, depth = self.pose_estimation(img, depth_image, intrinsic_camera, distortion)
            cv2.imshow('Estimated Pose', output)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            return ret, ids, depth

    def pose_estimation(self,frame,depth_frame, matrix_coefficients, distortion_coefficients):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(self.aruco_dict,parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        print(f"Pose Estimation Called, ids are {ids}")
        ret = False
        depth =0
        rotate_angle = 0
        if ids is not None and len(ids) > 0:
            object_points = np.array([[0, 0, 0],
                                    [0, 1, 0],
                                    [1, 1, 0],
                                    [1, 0, 0]], dtype=np.float32)
            ret = True
            for i in range(len(ids)):
                _, rvecs, tvecs = cv2.solvePnP(object_points, corners[i][0],matrix_coefficients,distortion_coefficients)
                # Draw detected markers
                aruco.drawDetectedMarkers(frame.copy(), corners)
                # Draw coordinate axes
                axis_length = 0.1  # Length of the axis in meters
                axis_points = np.float32([[0,0,0], [axis_length,0,0], [0,axis_length,0], [0,0,-axis_length]]).reshape(-1,3)
                image_points, _ = cv2.projectPoints(axis_points, rvecs, tvecs, matrix_coefficients, distortion_coefficients)
                frame = cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvecs, tvecs, axis_length)
                frame = aruco.drawDetectedMarkers(frame, corners, ids)
                #print(f"Rvecs = {rvecs}, Tvecs = {tvecs}")
                #print(f"Corners = {corners}")
               
                a=self.rotation_vector_to_euler_angles(rvecs,tvecs)
                #print(f"a = {a}")

                if a[0][0]<0:
                    angle_to_turn = 180+a[0][0]
                    print("Angle to Rotate", 180+a[0][0])
                else:
                    angle_to_turn = (180-a[0][0])*(-1)
                    print("Angle to Rotate: ",(180-a[0][0])*(-1))

                '''
                depth = depth_frame.get_distance((corners[0][0]+corners[1][0]+corners[2][0]+corners[3][0])/4,(corners[0][1]+corners[1][1]+corners[2][1]+corners[3][1])/4)
                '''
            '''
            if depth<=1.5:
                rotate_angle = 90 - angle_to_turn
            '''
        return ret, frame, ids, depth
    

    def detectAruco(self):
        model = YOLO("/home/kavin/Downloads/aruco_dataset/weights/best.pt")
        while True:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            depth = []
            if not color_frame or not depth_frame:
                continue
            img = np.asanyarray(color_frame.get_data())
            results = model.predict(img, conf = 0.5, max_det = 3)
            for r in results:
                annotator = Annotator(img)
                boxes = r.boxes
                for box in boxes:
                    b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                    c = box.cls
                    annotator.box_label(b, model.names[int(c)])

                    # Get dep/home/kavin/caesar2020_nvidia/src/navigation/scripts/kavin_modifi_2.pyth data for the bounding box
                    left, top, right, bottom = map(int, b)
                    arrow_center = (left+right)/2
                    try:
                        if (depth_frame.get_distance((left + right) // 2, (top + bottom) // 2)) != 0.0:
                            depth.append(depth_frame.get_distance((left + right) // 2, (top + bottom) // 2))
                    except:
                        ret = False
                    print("Depth for box" + str(b) + ":" + str(depth) +"meters")

            cv2.imshow('YOLO V8 Detection', img)   
            if cv2.waitKey(1) & 0xFF == ord(' '):
                break         
            
            #print("arrow in center: ", arrow_in_center)  
            #if arrow_in_center:
                #cv2.putText(img, "Arrow in centre", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            print("Depth from more than 3:", depth)
            return ret, arrow_center, depth, len(results)
            #else:
            return False, "Not available", None, 2.5




    def main(self):
        print("hi")
        self.ret, x, self.depth, self.length = self.detectAruco()
        print("hi")
        msg = WheelRpm()

        if self.ret == True:
            ret, frame, ids, depth, rotate_angle = self.aruco_recognition()
            if ret == True:
                self.ret == True
            else:
                self.ret == False
        if self.ret == True and self.length >= 2 and self.depth>=2:
            msg.vel = 102
            msg.omega = 127
            msg.hb = False
            publisher.publish(msg)
        if self.ret == True and self.length >=2 and self.depth < 2:
            msg.vel = 127
            msg.omega = 127
            msg.hb = False
            publisher.publish(msg)

    def state_callback(self,msg):
        self.state = msg.data

    def yaw_callback(self,msg):
        if (self.initial_drift_angle==0):   #only for 1st time
            self.initial_drift_angle=self.z_angle
        self.z_angle = msg.data

    def enc_callback(self,msg):
        self.enc_data = msg.data

    def gps_callback(self,msg):
        if(msg.latitude and  msg.longitude):
            self.current_latitude=msg.latitude
            self.current_longitude=msg.longitude
            file_object=open("coordinates.txt","a")
            file_object.write(f"latitude :{msg.latitude}, longitude :{msg.longitude} \n")
            file_object.close()



    def spin(self):
        while not rospy.is_shutdown():
            if(self.state==True):
                self.main()
                rate.sleep()

            else:
                print("Rover in Joystick mode")
                self.initial_drift_angle=self.z_angle
                rate.sleep()
  



if __name__== '__main__':
    rospy.init_node("hi", anonymous=True)
    publisher = rospy.Publisher("/arcvid", Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)
    run = Aruco_detect()
    run.spin()
    

