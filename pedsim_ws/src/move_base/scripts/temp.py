#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, Image

def pointcloud_callback(msg):
    timestamp = msg.header.stamp
    rospy.loginfo("[PointCloud2] Timestamp: %d.%09d" % (timestamp.secs, timestamp.nsecs))

def image_callback(msg):
    timestamp = msg.header.stamp
    rospy.loginfo("[Image] Timestamp: %d.%09d" % (timestamp.secs, timestamp.nsecs))

def listener():
    rospy.init_node('dual_topic_listener', anonymous=True)

    rospy.Subscriber("/mid360_PointCloud2_filtered", PointCloud2, pointcloud_callback)
    rospy.Subscriber("/camera/camera/color/image_raw", Image, image_callback)

    rospy.loginfo("Listening to /mid360_PointCloud2_filtered and /camera/camera/color/image_raw...")
    rospy.spin()

if __name__ == '__main__':
    listener()
