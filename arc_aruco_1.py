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
from ultralytics import YOLO
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics.utils.plotting import Annotator
from collections import defaultdict
from collections import OrderedDict
import cv2.aruco as aruco
from cv2.aruco import Dictionary

#This code is written for realsense camera
class Aruco_detection():

    def __init__(self):
        rospy.on_shutdown(self.stop_run)
        self.pipeline = rs.pipeline()
        config = rs.config()
        rospy.init_node("arrowdetectmorethan3")
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
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

        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10)

#change topics acc to requirements
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
        

    def rotation_vector_to_euler_angles(rvec,tvec):
        # Convert rotation vector to rotation matrix
        rotation_mat, _ = cv2.Rodrigues(rvec)
        pose_mat = cv2.hconcat((rotation_mat, tvec))
        _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(pose_mat)
        return euler_angles

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
            ret1, img = color_image.read()
            ret, output, ids, depth = self.pose_estimation(img, self.aruco_dict, intrinsic_camera, distortion)
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
        if ids is not None and len(ids) > 0:
            object_points = np.array([[0, 0, 0],
                                    [0, 1, 0],
                                    [1, 1, 0],
                                    [1, 0, 0]], dtype=np.float32)
            ret = True
            for i in range(len(ids)):
                _, rvecs, tvecs = cv2.solvePnP(object_points, corners[i][0],matrix_coefficients,distortion_coefficients)
                # Draw detected markers
                aruco.drawDetectedMarkers(frame, corners)
                # Draw coordinate axes
                axis_length = 0.1  # Length of the axis in meters
                axis_points = np.float32([[0,0,0], [axis_length,0,0], [0,axis_length,0], [0,0,-axis_length]]).reshape(-1,3)
                image_points, _ = cv2.projectPoints(axis_points, rvecs, tvecs, matrix_coefficients, distortion_coefficients)
                frame = cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvecs, tvecs, axis_length)
                frame = aruco.drawDetectedMarkers(frame, corners, ids)
                a=self.rotation_vector_to_euler_angles(rvecs,tvecs)
                if a[0][0]<0:
                    angle_to_turn = 180+a[0][0]
                    print(180+a[0][0])
                else:
                    angle_to_turn = (180-a[0][0])*(-1)
                    print((180-a[0][0])*(-1))
                depth = depth_frame.get_distance((corners[0][0]+corners[1][0]+corners[2][0]+corners[3][0])/4,(corners[0][1]+corners[1][1]+corners[2][1]+corners[3][1])/4)
            if depth<=1.5:
                rotate_angle = 90 - angle_to_turn

        return ret, frame, ids, depth, rotate_angle
    

    def detectAruco(self,color_image):
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

    def search(self):
        
        if abs(self.enc_data<4 and self.ret and not self.init):
            return
        
        print("Search function is active rn")
        if time.time() - self.start_time < 25:
            print("search will occur once this becomes 25:", time.time()-self.start_time)
            return
        
        msg1 = WheelRpm()
        msg1.hb = False
        msg1.omega = 127
        msg1.vel = 127
        wheelrpm_pub.publish(msg1)
        print("Rover has stopped")
        msg = std_msgs.Int32MultiArray()
        msg.data = [0,0,0,0,0,0]
        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0
        msg.layout.dim = [std_msgs.MultiArrayDimension()]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = 'write'
        self.pub.publish(msg)

        print("Entered While Loop")
        while self.init == False and abs(self.enc_data) < abs(self.start_angle) - 2*self.angle_thresh:
            msg.data = [0,255,0,0,0,0]
            rate.sleep()
            self.pub.publish(msg)
            self.start_time = time.time() - 4
        msg.data = [0,0,0,0,0,0]
        print("Exited while loop")
        
        self.init = True
        if self.init == True and abs(self.enc_data)<abs(self.start_angle) - 2 and not self.ret:
            print("Camera moving")
            msg.data = [0,-255,0,0,0,0]
            rate.sleep()
            self.pub.publish(msg)

        elif self.init == True and abs(self.enc_data)<abs(self.start_angle) and self.ret:
            self.distance = float(round(self.distance))
            for i in len(self.distance):
                self.angles_dict[self.distance[i]].append(self.enc_data)
                print("I'm appending to angles_dict")
            msg.data = [0,-255,0,0,0,0]
            self.pub.publish(msg)
            rate.sleep()
            print("Aruco detected at: ", self.enc_data)

        elif not self.ret:
            while abs(self.enc_data) > 4:
                if (self.enc_data) > 0:
                    msg.data = [0,-255,0,0,0,0]
                else:
                    msg.data = [0,255,0,0,0,0]
                rate.sleep()
                self.pub.publish(msg)
                self.init = False
                self.distance = 10.0
                self.start_time = time.time()
                
#idea 1: after stopping at one place, just search again. 



    def process_dict(self):
        print("The dictionary with dist:[enc_angles] : ", self.angles_dict)
        self.searchcalled = False
        max_length_key = 0.0

        try:
            print("Entered try of process_dict")
            if min(self.angles_dict.keys()) != 0:
                self.angles_dict = OrderedDict(sorted)
                self.list1 = []
                self.list2 = []
                list1_bool = True
                for i in (0,len(self.angles_dict),1):
                    if i == 0:
                        self.list1.append(sum(self.angles_dict[i])/len(self.angles_dict[i]))
                    d = self.angles_dict[i+1] - self.angles_dict[i]
                    if d < 2 and list1_bool == True:
                        self.list1.append(sum(self.angles_dict[i])/len(self.angles_dict[i]))
                    if d > 2:
                        list1_bool = False
                    if d < 2 and list1_bool == False:
                        self.list2.append(sum(self.angles_dict[i])/len(self.angles_dict[i]))
        except:
            print("The list is empty. No min value")
            self.init = False
            self.searchcalled = False
            return
        if len(self.list1)!=0 and len(self.list2)!=0:
                self.which_enc_angle_to_turn = (sum(self.list1)/len(self.list1) + sum(self.list2)/len(self.list2))/2
                self.where_am_i = 0
                self.tmp_dict = self.angles_dict
        elif len(self.list1)!=0 and len(self.list2)==0:
            self.which_enc_angle_to_turn = sum(self.list1)/len(self.list1)
            self.where_am_i = 1
        
        
        print("Angle to turn: ", self.which_enc_angle_to_turn)
        print("the dictionary with dist[enc_angles]: " ,self.angles_dict)

        if self.which_enc_angle_to_turn < 0:
            self.direction = "left"
            self.rotate_angle = abs(self.which_enc_angle_to_turn)

        else:
            self.direction = "right"
            self.rotate_angle = abs(self.which_enc_angle_to_turn)

        self.turn = True
        self.initial_yaw = self.z_angle
        self.angles_dict = defaultdict(list)


    def main(self):
        crab_motion_pub.publish(0)
        if (self.ret == False and self.turn == False) or self.init:
            self.search()
            if self.searchcalled:
                self.process_dict()
        if not self.turn:
            self.ret, arucoid, self.distance, no_arucos = self.detectAruco()
            if self.distance<4:
                ret, pix, self.arucoid, distance = self.aruco_recognition()
                if ret == True:
                    self.ret = True
                    self.distance = distance
                if self.ret:
                    print("Aruco detected at distance:", self.distance)
                    print("Direction:")
                else:
                    print("Trying to find aruco ....")
                    self.move_straight()
        else: # so self.turn = True in this case. 
            #This iteration's idea is to stop at the closest point and do search 
            print("I'm inside the ROTATE block")
            self.search()
            if self.searchcalled:
                self.process_dict()
            if not self.turn:
                self.ret, arucoid, self.distance, no_arucos = self.detectAruco()
                if self.distance<4:
                    ret, pix, self.arucoid, distance = self.aruco_recognition()
                    if ret == True:
                        self.ret = True
                        self.distance = distance

                if self.where_am_i == 0:
                    if self.direction == "left":
                        self.rotate(1)
                    elif self.direction == "right":
                        self.rotate(-1)   
                elif self.where_am_i == 1:
                    #now inspect the temporary memory space
                    self.dire_attempt = 1
                    list1 = []
                    list2 = []
                    list1_bool = []
                    print(self.tmp_dict)
                    for i in (0,len(self.tmp_dict),1):
                        if i == 0:
                            list1.append(sum(self.tmp_dict[i])/len(self.tmp_dict[i]))
                        d = self.tmp_dict[i+1] - self.tmp_dict[i]
                        if d < 2 and list1_bool == True:
                            list1.append(sum(self.tmp_dict[i])/len(self.tmp_dict[i]))
                        if d > 2:
                            list1_bool = False
                        if d < 2 and list1_bool == False:
                            list2.append(sum(self.tmp_dict[i])/len(self.tmp_dict[i]))
                        avg1_tmp = sum(list1)/len(list1)
                        avg2_tmp = sum(list2)/len(list2)
                        avg1_ang = sum(self.list1)/len(self.list1)
                        if avg1_tmp > avg1_ang:
                            #move left in crab motion
                            crab_motion_pub.publish(1)
                        if avg1_tmp < avg1_ang:
                            #move right in crab motion
                            crab_motion_pub.publish(-1)
                        start_time = time.time()
                        while time.time()-start_time < 5:
                            wheelrpm_pub.publish(102)
                        wheelrpm_pub.publish(127)
                        crab_motion_pub.publish(0)
                        self.move_straight()
                        
                    
    def move_straight(self):
        msg = WheelRpm()
        msg.hb = False

        if (abs(self.initial_drift_angle-self.z_angle) > 10 and self.searchcalled == False):    #only if large drift (of 10 degrees) is there, correct it.
            while (abs(self.initial_drift_angle-self.z_angle) > 6):    
                #while it doesn't come back to near the proper orientation, keep giving omega

                msg.omega=int(127+self.kp_straight_rot*(self.initial_drift_angle-self.z_angle))
                msg.omega+=10*int((self.initial_drift_angle-self.z_angle)/abs(self.initial_drift_angle-self.z_angle))   #in case it can't rotate fast    # c*x/abs(x) is basically c*(sign of x)
                
                #capping velocity
                if(msg.omega<95):
                    msg.omega = 95
                elif(msg.omega>159):
                    msg.omega = 159
                msg.vel=127
                print("correcting drift with omega =", msg.omega)
                wheelrpm_pub.publish(msg)
                rate.sleep()
            rospy.sleep(1)
        
        msg.omega=127
        if self.init or self.searchcalled:
            msg.vel = 127
            wheelrpm_pub.publish(msg)
        
        elif self.ret:
            if self.distance>2:
                msg.vel = 102 
                wheelrpm_pub.publish(msg)
            else:
                print("Nearing the aruco")
                if self.where_am_i == 0:
                    msg.vel == 102
                    wheelrpm_pub.publish(msg)
                #THE GREY REGION
                if self.where_am_i == 1 and self.dire_attempt == 1:
                    msg.vel == 112         
                     
    
    def spin(self):
        while not rospy.is_shutdown():
            if(self.state==True):
                self.main()
                rate.sleep()

            else:
                print("Rover in Joystick mode")
                self.initial_drift_angle=self.z_angle
                rate.sleep()
  

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
            wheelrpm_pub.publish(msg)
        else:
            msg.omega=127
            msg.vel = 127
            wheelrpm_pub.publish(msg)
            # self.z_angle=0

            self.initial_drift_angle=self.z_angle
            print("****ROTATE DONE*****")
            #self.distance = 10.0
            self.start_time = time.time()-10
            self.turn=False
            self.direction = "Not Available"
            rospy.sleep(2)
    

    def stop_run(self):
        msg_stop=WheelRpm()
        msg_stop.hb=False
        msg_stop.vel=msg_stop.omega=127
        wheelrpm_pub.publish(msg_stop)
#        self.bag.close()
    




if __name__ == '__main__':
    try:
        rospy.init_node("aruco_detection_arc")
        rate = rospy.Rate(10)
        wheelrpm_pub=rospy.Publisher('motion',WheelRpm,queue_size=10)
        gps_data_pub = rospy.Publisher('gps_bool',std_msgs.Int8,queue_size=10)
        crab_motion_pub = rospy.Publisher('crab_bool', std_msgs.Int8, queue_size=10)
        run = Aruco_detection()
        run.spin()
    except KeyboardInterrupt:
        sys.exit()
