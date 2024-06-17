#!/usr/bin/env python3
import rospy
import numpy as np
import cv2 as cv
import time
from scipy import stats
from std_msgs.msg import Float64MultiArray
import pyrealsense2 as rs

def cam(): 
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    total_disp = []

    # Wait until color image is available
    color_image = None
    while color_image is None:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
        else:
            rospy.loginfo("Waiting for color image...")
            rospy.sleep(1)  # Wait for 1 second before trying again
    
    feature_params = dict(maxCorners = 50, qualityLevel=0.1, minDistance=1, blockSize=1)
    lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
    color = np.random.randint(0, 255, (100, 3))
    old_gray = cv.cvtColor(color_image, cv.COLOR_BGR2GRAY)
    mask = np.zeros_like(color_image)
    t0 = time.time()
    ti = time.time()
    t2 = time.time()
    p0 = cv.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

    rospy.init_node('cam_in', anonymous=True)
    pub = rospy.Publisher('cam_vel', Float64MultiArray, queue_size=10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        time_elapsed = time.time() - t0
        if time_elapsed > 1:
            p0 = cv.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
            t0 = time.time()
            mask = np.zeros_like(color_image)
        
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        frame = np.asanyarray(color_frame.get_data())
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        if p1 is not None:
            good_new = p1[st == 1]
            good_old = p0[st == 1]
    
        # x_dis = good_new[:, 0] - good_old[:, 0]
        # y_dis = good_new[:, 1] - good_old[:, 1]
        # x_dis_int = np.array(x_dis, dtype=np.int32)
        # y_dis_int = np.array(y_dis, dtype=np.int32)
        # x_mode = stats.mode(x_dis_int)[0]
        # y_mode = stats.mode(y_dis_int)[0]
        # x_dis_before_average = x_dis[x_dis_int == x_mode]
        # y_dis_before_average = y_dis[y_dis_int == y_mode]
        # x_dis_avg = np.nanmean(x_dis_before_average)
        # y_dis_avg = np.nanmean(y_dis_before_average)
        frame_displacement_sum = 0.0
        if (time.time() - ti < 3):
            x_dis_avg = 0
            y_dis_avg = 0
            
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            depth_frame = frames.get_depth_frame()
            if 0 <= b < depth_frame.height and 0 <= a < depth_frame.width and 0 <= d < depth_frame.height and 0 <= c < depth_frame.width:
                # Calculate depth difference
                depth1 = depth_frame.get_distance(int(a), int(b))
                depth2 = depth_frame.get_distance(int(c), int(d))
                if depth1 == 0.0 or depth2 == 0.0:
                    continue
                displacement = depth1 - depth2
                if abs(displacement) > 0.1:
                    continue
                if abs(displacement) <0.0001:
                    continue
                frame_displacement_sum += displacement            
            mask = cv.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
            frame = cv.circle(frame, (int(a), int(b)), 5, color[i].tolist(), -1)

        img = cv.add(frame, mask)
        cv.imshow('frame', img)
        k = cv.waitKey(30) & 0xff
        if k == 27:
            break

        print("frame disp sum", frame_displacement_sum/(i+1))
        print("i :", i+1)
        if (frame_displacement_sum)/(i+1)>0.005:

            total_disp.append(frame_displacement_sum/(i+1))
            print("appending to the master dict")
        if len(total_disp) != 0:
            print("displacement so far: ", sum(total_disp)) 
            print("len(total_disp) = ", len(total_disp))
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1, 1, 2)
        # del_t = time.time() - t2
        # t2 = time.time()
        # vel_x = x_dis_avg / (del_t * 100)
        # vel_y = y_dis_avg / (del_t * 100)
       
        # vel_msg = Float64MultiArray()
        # vel = [vel_x, vel_y]
        # vel_msg.data = vel
        # pub.publish(vel_msg)
        r.sleep()

    pipeline.stop()
    cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        cam()
    except rospy.ROSInterruptException:
        pass
