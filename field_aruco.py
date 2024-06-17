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


class Aruco_Simulation():
    def __init__(self):
        print("Initializing...")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/intel/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/intel/depth/image_raw", Image, self.depth_callback)
        self.galileo_pub = rospy.Publisher("/galileo/cmd_vel", Twist, queue_size=10)
        self.window_name = "ARC Aruco Simulation"
        self.cv_image = None
        self.depth_image = None

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            print(e)

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except Exception as e:
            print(e)

    def rotation_vector_to_euler_angles(self, rvec, tvec):
        rotation_mat, _ = cv2.Rodrigues(rvec)
        pose_mat = cv2.hconcat((rotation_mat, tvec))
        _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(pose_mat)
        return euler_angles

    def aruco_recognition(self):
        if self.cv_image is None:
            return False, None, None, None

        color_image = self.cv_image
        output = None
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
        # intrinsic_camera=np.array(((607.7380981445312, 0.0, 325.2829284667969),(0.0, 606.8139038085938, 238.5009307861328),(0,0,1.0)))
        intrinsic_camera=np.array(((1056.86751311741, 0.0, 640.5),(0.0, 1056.86751311741, 360.5),(0,0,1.0)))
        distortion=np.array((0.0, 0.0, 0.0, 0.0, 0.0))
        if color_image.any()!= None:
            ret, output, ids, depth = self.pose_estimation(intrinsic_camera, distortion)
            if output != None:
                cv2.imshow('Estimated Pose', output)
            key = cv2.waitKey(1) & 0xFF
        return ret, None, None, depth

    def detectAruco(self):
        if self.cv_image is None:
            return False, None, None, None

        model = YOLO("/home/kavin/Downloads/aruco_dataset/weights/best.pt")
        depth = 100.0
        img = np.asanyarray(self.cv_image)
        results = model.predict(img, conf=0.1)
        ret = False

        for r in results:
            annotator = Annotator(img)
            boxes = r.boxes
            for box in boxes:
                ret = True
                b = box.xyxy[0]
                c = box.cls
                annotator.box_label(b, model.names[int(c)])

                left, top, right, bottom = map(int, b)
                depth = self.depth_image[(left + right) // 2, (top + bottom) // 2]
                print(f"Depth of Aruco: {depth}")
        print("ret from model:", ret)
        cv2.imshow('YOLO V8 Detection', img)
        cv2.waitKey(1)

        return ret, None, depth, len(results)
    

    def pose_estimation(self, matrix_coefficients, distortion_coefficients):
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(self.aruco_dict,parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        print(f"Pose Estimation Called, ids are {ids}")
        ret = False
        depth =0
        rotate_angle = 0
        frame = None
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

                
                # depth = self.depth_image((corners[0][0]+corners[1][0]+corners[2][0]+corners[3][0])/4,(corners[0][1]+corners[1][1]+corners[2][1]+corners[3][1])/4)
                
            '''
            if depth<=1.5:
                rotate_angle = 90 - angle_to_turn
            '''
        return ret, frame, ids, depth


    def main(self):
        twist = Twist()
        print("Depth_image:", type(self.depth_image))
        ret1, _, depth1, _ = self.detectAruco()
        ret2,_,_1, depth2 = self.aruco_recognition()

        if ((ret1 or ret2) and depth1 > 2) or (not ret1 and not ret2):
            twist.linear.x = 0.1
        if (ret1 or ret2) and depth1 < 2:
            twist.linear.x = 0


        self.galileo_pub.publish(twist)

    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()
        else:
            print("Waiting")


if __name__ == '__main__':
    try:
        rospy.init_node("hi", anonymous=True)
        rate = rospy.Rate(10)
        ahh = Aruco_Simulation()
        ahh.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()