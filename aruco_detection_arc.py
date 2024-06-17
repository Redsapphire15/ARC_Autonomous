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

class auto():

    def __init__(self):
        
        #self.cap=cv2.VideoCapture(0)
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
        self.template_r=cv2.imread('Template.png',0)
        self.template_l=cv2.imread('Template_l.png',0)
        self.template_r=cv2.resize(self.template_r,(60,40),cv2.INTER_AREA)
        self.template_l=cv2.resize(self.template_l,(60,40),cv2.INTER_AREA)
        self.h,self.w=self.template_r.shape
        self.z_angle = self.x_angle = self.y_angle = 0
        self.turn = False
        self.circle_dist=2.0
        self.dist_thresh=0.3
        self.angle_thresh=4
        self.kp=20
        self.kp_rot=1.5
        self.kp_straight_rot=3
        self.distance=10.0
        for i in range(5):
            print("hey! self.distance = 10",self.distance)
        self.direction="Not Available"
        self.current_latitude=0.0
        self.current_longitude=0.0
        self.ret=False
        self.initial_yaw=0.0
        self.rotate_angle = 90
        self.angles_dict = defaultdict(list)
        self.searchcalled = False
        self.latlong = defaultdict(list)
        self.latlong[0] = "latitude"
        self.latlong[1] = "longitude"
        self.arrow_numbers = 5
        self.gpscalled =0
        self.where_am_i = 100

        #bag
#        self.num=i
#        filename = "imu_data_"+str(self.num)+".bag"
#        self.bag=rosbag.Bag(filename,'w')
        self.state = False
        self.initial_drift_angle=0

        #search alg by turning realsense
        self.enc_data=0
        self.start_time=time.time()
        self.time_thresh = 20
        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10)
        self.init = False
        self.start_angle = 55
        self.angle_thresh = 4
        #self.manjari = False
        self.count_arrow = 0
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
                        depth = depth_frame.get_distance((left + right) // 2, (top + bottom) // 2)
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
            return ret, arrow_center, depth
            #else:
            return False, "Not available", None, 2.5
            
                



#self.ret,self.direction,pix,self.distance
    def search(self):
        #print("self.searchcalled (should print false here always):",self.searchcalled)     # prints False always because it is set to False in process_dict, which always happens when searchcalled is true (before coming back to search)
        if(abs(self.enc_data) < 0.6*self.angle_thresh and self.ret and not self.init):   
            # if arrow is detected and realsense is facing straight, then come out of search immediately.
            return

        print("Search() has been called.")
        #self.searchcalled = True
        print("time.time():",time.time())
        print("self.start_time:",self.start_time)
        if time.time() - self.start_time < self.time_thresh:    #time_thresh is 20s
            print("time.time()-self.start_time (when this becomes 20s, search will happen):",time.time()-self.start_time)
            return
        msg1 = WheelRpm()
        msg1.hb = False
        msg1.omega = 127
        msg1.vel = 127
        wheelrpm_pub.publish(msg1)
        print("Rover has stopped.")
        msg = std_msgs.Int32MultiArray()
        msg.data=[0,0,0,0,0,0]
        
        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0

        msg.layout.dim = [ std_msgs.MultiArrayDimension() ]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = 'write'
        self.pub.publish(msg)

        print("Entered while loop.")
        while self.init == False and abs(self.enc_data) < abs(self.start_angle)-2*self.angle_thresh:   
            #to make the realsense go to the 60 degree maximum before starting the burst search

            msg.data = [0,255,0,0,0,0]
            rate.sleep()
            self.pub.publish(msg)
            self.start_time = time.time() - self.time_thresh
        msg.data = [0,0,0,0,0,0]
        print ("Exited while loop.")
        self.init = True
        #print("self.init (set to true in the previous line:",self.init)
        print("Realsense's angle:", self.enc_data)
        print("self.ret:", self.ret)
        if self.init == True and abs(self.enc_data) < (abs(self.start_angle)-0.5*self.angle_thresh) and not self.ret:
            # if arrow is not detected and the realsense has not gone beyond the 60 degree maximum, continue moving realsense
            # self.init is not made false, so this will happen again when search is called in main()

            print("Camera Moving")
            msg.data = [0,-255,0,0,0,0]
            rate.sleep()
            self.pub.publish(msg)
            print()
            #main area
        elif self.init == True and abs(self.enc_data) < abs(self.start_angle) and self.ret:
            #if arrow is detected and the realsense is within the 60 degree maximum, append the arrow's values and continuemoving realsense
            # self.init is not made false, so this will happen again when search is called in main()

            #self.distance = self.distance*1
            #self.distance = self.distance // 1
            #self.distance = self.distance / 1
            self.distance = float(round(self.distance))
            print("distance before appending: ",self.distance)
            if self.distance != 0.0:                                              #change
                self.angles_dict[self.distance].append(self.enc_data) 
                print("I'm appending to angles_dict")
            msg.data = [0,-255,0,0,0,0]
            self.pub.publish(msg)
            rate.sleep()
            print("Arrow found at: ", self.enc_data)
            print()
        elif not self.ret:
            # when the realsense has crossed the 60 degree maximum, realsense comes back to middle and the dictionary is processed
            # self.init is set to false (when next search() is called, realsense will first move to the 60 degree maximum)
            # and the counting of time is reset (that is, the next search will happen at least 20s after this block of code)

            while abs(self.enc_data) > self.angle_thresh :
                if(self.enc_data > 0):
                    msg.data = [0,-255,0,0,0,0]
                else:
                    msg.data = [0,255,0,0,0,0]
                rate.sleep()
                self.pub.publish(msg)
            msg.data = [0,0,0,0,0,0]
            self.pub.publish(msg)
            self.init = False
            self.searchcalled = True
            self.distance = 10.0
            self.start_time = time.time()
        '''     
            while self.enc_data >5:
                pass
                #go in one direction to 0.
            while self.enc_data < -5:
                pass
                #go in other direction to 0.
            return
        '''
    def process_dict(self):
        #print("self.searchcalled (should print true always):", self.searchcalled)
        if not self.init and self.searchcalled:
            # the first part is not needed, because whenever self.searchcalled is set to true, self.init is set to false
            # actually, this if is only not needed, because in main(), there is an if self.searchcalled(), then process_dict(), which takes care of everything

            print("the dictionary with dist:[enc angles] :- ",self.angles_dict)
            self.searchcalled = False
            max_length_key = 0.0
        
            try:
                print("Entered Try of process_dict")
                #if min(self.angles_dict.keys()) == '0.0':
                    #del self.angles_dict['0.0']
                if min(self.angles_dict.keys()) != 0:
                    print("Entered if")
                    #del self.angles_dict['0.0']]
                    self.angles_dict = OrderedDict(sorted)
                    list1 = []
                    list2 = []
                    list1_bool = True
                    for i in (0,len(self.angles_dict),1):
                        if i == 0:
                            list1.append(sum(self.angles_dict[i])/len(self.angles_dict[i]))
                        d = self.angles_dict[i+1] - self.angles_dict[i]
                        if d < 2 and list1_bool == True:
                            list1.append(sum(self.angles_dict[i])/len(self.angles_dict[i]))
                        if d > 2:
                            list1_bool = False
                        if d < 2 and list1_bool == False:
                            list2.append(sum(self.angles_dict[i])/len(self.angles_dict[i]))

                    #max_length_key = max(self.angles_dict, key=lambda k: len(self.angles_dict[k]))
            except:
                print("The list is empty. No minimum value.")
                self.init = False
                self.searchcalled = False
                return
            if len(list1)!=0 and len(list2)!=0:
                self.which_enc_angle_to_turn = (sum(list1)/len(list1) + sum(list2)/len(list2))/2
                self.where_am_i = 0
            elif len(list1)==0 and len(list2)!=0: 
                self.which_enc_angle_to_turn = sum(list2)/len(list2)
                self.where_am_i = -1
            elif len(list1)!=0 and len(list2)==0:
                self.which_enc_angle_to_turn = sum(list1)/len(list1)
                self.where_am_i = 1
            

            print("Angle to turn:", self.which_enc_angle_to_turn)
            print("the dictionary with dist:[enc angles] :- ",self.angles_dict)

            #encoder need not be perfect. if in case there is some cup, edit this angle as per your needs
            if (self.which_enc_angle_to_turn<0):
                self.direction="left"
                #self.rotate_angle=abs(self.which_enc_angle_to_turn + 0.5*self.angle_thresh) #+2 degrees
                self.rotate_angle=abs(self.which_enc_angle_to_turn)
            else:
                self.direction="right"
                #self.rotate_angle=(self.which_enc_angle_to_turn - 0.5*self.angle_thresh) #+2 degrees
                self.rotate_angle=abs(self.which_enc_angle_to_turn)
        
            self.turn = True
            self.initial_yaw=self.z_angle 
            self.angles_dict = defaultdict(list)

    
    def main(self):
        if (self.ret == False and self.turn == False) or (self.init) :
            self.search()
            if (self.searchcalled):
                self.process_dict()
        if not self.turn :
            self.ret, arucoid, self.distance = self.detectAruco()
            if self.distance<4.0:
                ret, pix, arucoid, distance = self.aruco_recognition()
                if ret == True:
                    self.ret = ret
                    self.arucoid = arucoid
                    self.distance = distance
            if (self.ret):
                print("Aruco detected at distance: "+str(self.distance))
                print("Direction: " + self.direction)
            else:
                print("Trying to detect aruco...")
                self.move_straight()

        else:   #if self.turn is true, everything stops and rover does only turning
            print("Im going into rotate block in main")
            if(self.direction=="left"):
                print("rotating left")
                self.rotate(1)
            elif (self.direction=="right"):
                print("rotating right")
                self.rotate(-1)        

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

    def move_straight(self):
        msg = WheelRpm()
        msg.hb=False
        
        #Aadit's p controller
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

        # if(self.init or self.searchcalled):
        #     print("move_straight() is being ignored due to search().")
        #     msg.vel=127
        #     wheelrpm_pub.publish(msg)
        # elif(self.ret):
        #     if(abs(self.circle_dist-self.distance)>self.dist_thresh):
        #         if self.distance!=0.0 and self.distance!=2.5:
        #             msg.vel=max(102,int(127+self.kp*(self.circle_dist-self.distance)))
        #             print("Moving straight. ",(self.circle_dist-self.distance))
        #             wheelrpm_pub.publish(msg)
        #         else:
        #             msg.vel=90    #why??
        #             wheelrpm_pub.publish(msg)

        #     else:
        #         msg.vel=127
        #         msg.omega=127
        #         wheelrpm_pub.publish(msg)
        #         print("Stopped going Straight")
        #         self.gpscalled = 1
        #         gps_data_pub.publish(self.gpscalled)
        #         for i in range(100):
        #             gps_data_pub.publish(self.gpscalled)     
        #             rate.sleep()                               # 10s   # Competition rules say 10s                    
        #         if self.count_arrow < self.arrow_numbers:
        #             #rospy.sleep(10)
        #             #self.latlong[0].append(msg.latitude)
        #             #self.latlong[1].append(msg.longitude)
        #             self.gpscalled = 0
        #             print()
        #             print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        #             print("lat:",self.current_latitude) 
        #             print("long:",self.current_longitude)
        #             print()
        #             self.turn = True
        #             #right now the rover has identified itself before atleast one of the arucos
        #             if self.where_am_i == 0 :

        #             self.initial_yaw=self.z_angle
        #             self.count_arrow += 1
        #         else:
        #             self.v1_competition()

#                self.write_coordinates()

        # else:
        #     print("Forward")
        #     msg.vel = 102
        #     wheelrpm_pub.publish(msg)
        #     self.turn=False
        

        #didnt identify arucos till now
        if self.init or self.searchcalled:
            msg.vel = 127
            wheelrpm_pub.publish(msg)
        #identified aruco while moving straight
        elif self.ret:
            if self.distance > 2:
                print("search ignored")
                msg.vel = 102
                wheelrpm_pub.publish(msg)
                
            else:
                print("Im nearing the aruco which one idk")
                if self.where_am_i == 0:
                    print("I am on the mid path. Good to go in straightly and leave the rest to lidar")
                    msg.vel = 102
                    wheelrpm_pub.publish(msg)
                if self.where_am_i== -1 or self.where_am_i == 1:
                    #Here the rover just saw one aruco and waiting
                    self.searchcalled = True







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
        
        #self.manjari = False

    def stop_run(self):
        msg_stop=WheelRpm()
        msg_stop.hb=False
        msg_stop.vel=msg_stop.omega=127
        wheelrpm_pub.publish(msg_stop)
#        self.bag.close()
        
    def v1_competition(self):
        msg_stop=WheelRpm()
        msg_stop.hb=False
        msg_stop.vel=msg_stop.omega=127
        wheelrpm_pub.publish(msg_stop)
        print("Course completed(hopefully)")
        while not rospy.is_shutdown():
            if(self.state==True):
                print("Press 'A' to go to joystick mode.")
                rate.sleep()

            else:
                print("Rover in Joystick mode")
                rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node("arrowdetectmorethan3")
        rate=rospy.Rate(10)
#    i=int(input("Enter test number: "))
        wheelrpm_pub=rospy.Publisher('motion',WheelRpm,queue_size=10)
        gps_data_pub = rospy.Publisher('gps_bool',std_msgs.Int8,queue_size=10)
        run=auto()
        run.spin()
    except KeyboardInterrupt:
    # quit
        sys.exit()

