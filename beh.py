#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
# import sensor_msgs.msg



pub = rospy.Publisher('/revised_scan', Float32MultiArray, queue_size = 10)

rev_scan = Float32MultiArray()



def callback(msg):
    #print(len(msg.ranges)) len is 2019 from 0-360

    req_range = msg.ranges[0:1153]
    rev_scan.data = req_range
    # lidarmean_2 = req_range[923]
    # lidarmean_1 = req_range[1038]
    # lidarmean_0 = req_range[0]
    # lidarmean__1 = req_range[115]
    # lidarmean__2 = req_range[230]
    # lidarmean = [lidarmean_2,lidarmean_1,lidarmean_0, lidarmean__1,lidarmean__2]
    #print(len(req_range))
    # print(len(msg.ranges))
    lidarmean = []
    lidarmean.append(req_range[230])
    lidarmean.append(req_range[115])
    lidarmean.append(req_range[0])
    lidarmean.append(req_range[1038])
    lidarmean.append(req_range[923])
    print(lidarmean)
    
    pub.publish(rev_scan)

def listener():
    rospy.init_node('revised_scan', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
   