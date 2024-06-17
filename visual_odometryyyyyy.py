import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs
import rospy
from std_msgs.msg import Float32MultiArray

def vis_odom():

    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
    pipeline.start(config)

    # Parameters for Shi-Tomasi corner detection
    feature_params = dict(maxCorners=1000, qualityLevel=0.4, minDistance=1, blockSize=7)
    # Parameters for Lucas-Kanade optical flow
    lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

    # Variable for color to draw optical flow track
    color = (0, 255, 0)
    # Initialize total displacement
    total_displacements = []

    # Get the first frame to initialize prev_gray and prev points
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    prev_gray = cv.cvtColor(color_image, cv.COLOR_BGR2GRAY)
    prev = cv.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)

    # Initialize displacement for the current frame
    frame_displacement = 0
    frame_displacement_sum = 0

    rospy.init_node("odom", anonymous=True  )
    pub = rospy.Publisher("odom", Float32MultiArray,queue_size=10 )
    rate = rospy.Rate(30)
    # Depth scale factor
    #depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
    #depth_scale = 0.00266

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Convert color frame to numpy array
            color_image = np.asanyarray(color_frame.get_data())


            # Converts each frame to grayscale
            gray = cv.cvtColor(color_image, cv.COLOR_BGR2GRAY)

            # Finds the strongest corners in the current frame by Shi-Tomasi method
            current = cv.goodFeaturesToTrack(gray, mask=None, **feature_params)
            
            # If there are no previous points or no current points, skip optical flow calculation
            if prev is not None and current is not None:
                # Calculates sparse optical flow by Lucas-Kanade method
                next, status, error = cv.calcOpticalFlowPyrLK(prev_gray, gray, prev, None, **lk_params)

                print(error)
                # Create a mask for drawing optical flow tracks
                mask = np.zeros_like(color_image)

                # Selects good feature points for previous position
                good_old = prev[status == 1]
                if next.any() != None:
                # Selects good feature points for next position
                        good_new = next[status == 1]
                tmp = 0.0

                # Draws the optical flow tracks
                
                for i, (new, old) in enumerate(zip(good_new, good_old)):
                    # Returns a contiguous flattened array as (x, y) coordinates for new point
                    a, b = new.ravel().astype(int)
                    # Returns a contiguous flattened array as (x, y) coordinates for old point
                    c, d = old.ravel().astype(int)
                    # Draws line between new and old position with green color and 2 thickness
                    mask = cv.line(mask, (a, b), (c, d), color, 2)
                    # Draws filled circle (thickness of -1) at new position with green color and radius of 3
                    color_image = cv.circle(color_image, (a, b), 3, color, -1)
                    # Calculate displacement for this feature point
                    # displacement = np.sqrt((a - c) ** 2 + (b - d) ** 2)
                    print("a,b", a, b)
                    print("c,d",c,d)
                    print("height of depth frame", depth_frame.height)
                    print("width of depth frame", depth_frame.width)
                    if 0 <= b < depth_frame.height and 0 <= a < depth_frame.width and 0 <= d < depth_frame.height and 0 <= c < depth_frame.width:
                    # Calculate depth difference
                        d1 = depth_frame.get_distance(int(a), int(b))
                        
                        if d1 == 0.0:
                            continue
                    print("d1", d1, )
                    displacement = d1 - tmp
                    if displacement>2.0:
                        continue
                    print("displacement", displacement)
                    # Accumulate displacement to frame displacement
                    frame_displacement_sum = frame_displacement_sum + displacement
                    print("frame displacement", frame_displacement_sum)
                    # Accumulate frame displacement to total displacement
                    tmp = d1
                    
                    #print(depth_scale)
                # Overlays the optical flow tracks on the original frame
                output = cv.add(color_image, mask)

                # Updates previous frame and points
                prev_gray = gray.copy()
                prev = good_new.reshape(-1, 1, 2)
                total_displacements.append(frame_displacement_sum)
                # Display the frame
                cv.imshow('RealSense', output)
                print("Frame displacement (meters):", frame_displacement_sum/(i+1))
                #  print("Total displacement (meters):", sum(total_displacements)/(i+1))

                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print( f"Prev = {prev}, current = {current}")

    finally:
        # Stop streaming
        pipeline.stop()

    # Plot the total displacement over time
    plt.plot(total_displacements)
    plt.xlabel('Frame')
    plt.ylabel('Total Displacement (meters)')
    plt.title('Total Displacement of Camera Over Time')
    plt.show()

    # The following frees up resources
    cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        vis_odom()
    except rospy.ROSInterruptException:
        pass
        cv.destroyAllWindows()

