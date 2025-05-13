#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import tf.transformations
from geometry_msgs.msg import PointStamped, PoseStamped
from collections import deque

class TrackingObjectRelay:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('tracking_object_relay', anonymous=True)

        # Create publisher for "/move_base_simple/goal" topic (PoseStamped type)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Subscribe to "/tracking_object/position" topic (PointStamped type)
        self.sub = rospy.Subscriber('/tracking_object/position', PointStamped, self.callback)

        # Set 5Hz frequency
        self.rate = rospy.Rate(5)  # 5Hz = 0.2 second interval

        # Store the latest PointStamped message
        self.latest_msg = None
        
        # Store a history of position messages (deque for efficient operations)
        self.position_history = deque(maxlen=100)  # Store up to 100 recent positions
        
        # Minimum distance for orientation calculation (0.1 meters)
        self.min_distance = 0.1
        
    def callback(self, msg):
        """ Callback function to store the latest tracking target point """
        # Store current message as latest
        self.latest_msg = msg
        
        # Add to position history
        self.position_history.append(msg)

    def find_suitable_reference_point(self):
        """
        Find a previous point that is at least 0.1m away from the latest position
        to use for orientation calculation
        """
        if len(self.position_history) < 2:
            return None
            
        latest_point = self.latest_msg.point
        
        # Iterate through history from newest to oldest (excluding the latest)
        for past_msg in reversed(list(self.position_history)[:-1]):
            past_point = past_msg.point
            
            # Calculate distance between the latest and this past point
            distance = math.sqrt((latest_point.x - past_point.x)**2 + 
                                 (latest_point.y - past_point.y)**2 + 
                                 (latest_point.z - past_point.z)**2)
            
            # If distance is at least 0.1m, use this point
            if distance >= self.min_distance:
                return past_msg
        
        # If no point is far enough, return the oldest point we have
        return self.position_history[0]

    def calculate_orientation(self, current_point, previous_point):
        """
        Calculate orientation (yaw angle) based on the difference between
        current and previous points
        """
        # Calculate the difference in x and y coordinates
        dx = current_point.x - previous_point.x
        dy = current_point.y - previous_point.y
        
        # Calculate the yaw angle (in radians)
        yaw = math.atan2(dy, dx)
        
        # Convert yaw to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        
        return quaternion

    def run(self):
        """ Publish PoseStamped messages at 5Hz frequency """
        while not rospy.is_shutdown():
            if self.latest_msg is not None and len(self.position_history) > 1:
                # Find a suitable previous point for orientation calculation
                reference_msg = self.find_suitable_reference_point()
                
                if reference_msg is not None:
                    # Create PoseStamped message
                    pose_msg = PoseStamped()
                    pose_msg.header = self.latest_msg.header  # Inherit timestamp
                    pose_msg.pose.position = self.latest_msg.point  # Inherit position
                    
                    # Calculate orientation based on the difference between current and reference positions
                    quaternion = self.calculate_orientation(self.latest_msg.point, reference_msg.point)
                    
                    # Set orientation
                    pose_msg.pose.orientation.x = quaternion[0]
                    pose_msg.pose.orientation.y = quaternion[1]
                    pose_msg.pose.orientation.z = quaternion[2]
                    pose_msg.pose.orientation.w = quaternion[3]
                    
                    # Publish the converted message
                    self.pub.publish(pose_msg)
                    
                    rospy.logdebug(f"Published pose with orientation calculated from points {self.min_distance}m apart")
                else:
                    rospy.logwarn("Could not find suitable reference point for orientation calculation")
                    
            # Control loop frequency at 5Hz
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Start ROS node
        node = TrackingObjectRelay()
        node.run()  # Enter 5Hz publishing loop
    except rospy.ROSInterruptException:
        pass