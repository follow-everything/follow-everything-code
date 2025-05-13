#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import message_filters
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from dynamic_reconfigure.client import Client
from visualization_msgs.msg import Marker  # For RViz visualization

class VelocityController:
    def __init__(self):
        rospy.init_node('dynamic_velocity_controller', anonymous=True)

        # Kalman Filter Initialization (Holonomic Model)
        self.dt = 0.1  # Time step
        self.A = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        self.Q = np.diag([0.01, 0.01, 0.1, 0.1])
        self.R = np.diag([0.05, 0.05])
        self.P = np.eye(4)
        self.x = np.zeros((4, 1))

        # Initialize safety distance
        self.safety_distance = 1.5
        self.desired_velocity = 1.0
        self.accumulated_uncertainty = 0

        # Dynamic Reconfigure Client for teb_local_planner
        self.teb_client = Client('/move_base/TebLocalPlannerROS', timeout=500)

        # Publishers
        self.safety_distance_pub = rospy.Publisher('/tracking_object/safety_distance', Float32, queue_size=10)
        self.marker_pub = rospy.Publisher('/safety_marker', Marker, queue_size=10)  # For RViz visualization

        # Create synchronized subscribers
        position_sub = message_filters.Subscriber('/tracking_object/position', PointStamped)
        odom_sub = message_filters.Subscriber('/odom', Odometry)
        
        # Approximate Time Synchronizer (handles slight delays)
        sync = message_filters.ApproximateTimeSynchronizer([position_sub, odom_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self.sync_callback)


    def kalman_filter(self, z):
        """ Kalman filter update step for (x, y) tracking. """
        # Prediction step
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

        # Measurement update step
        y = z - np.dot(self.H, self.x)  # Measurement residual
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(self.P, self.H.T).dot(np.linalg.inv(S))  # Kalman Gain

        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.eye(4) - np.dot(K, self.H)), self.P)

        return self.x[0, 0], self.x[1, 0], np.trace(self.P)  # Estimated (x, y) and uncertainty

    def sync_callback(self, position_msg, odom_msg):
        """ Synchronized callback for both tracking object and robot odometry. """
        # Process object position with Kalman Filter
        measured_position = np.array([[position_msg.point.x], [position_msg.point.y]])
        estimated_x, estimated_y, uncertainty = self.kalman_filter(measured_position)

        object_position = position_msg.point
        object_position.x = estimated_x
        object_position.y = estimated_y

        robot_position = odom_msg.pose.pose.position

        # Adjust safety distance based on uncertainty
        if position_msg.point.z == -1:
            self.safety_distance = 0
            # rospy.loginfo(f"==========  Safety Distance: {self.safety_distance:.2f}")
        else:
            # self.safety_distance = 2.0 + (uncertainty / 2.0)  # Scale to [1.5, 2.5] range
            # self.safety_distance = np.clip(self.safety_distance, 2.0, 3.0)
            self.safety_distance = 1.5 + (uncertainty / 2.0)  # Scale to [1.5, 2.5] range
            self.safety_distance = np.clip(self.safety_distance, 1.5, 2.5)
            # self.safety_distance = 1.0 + (uncertainty / 2.0)  # Scale to [1.5, 2.5] range
            # self.safety_distance = np.clip(self.safety_distance, 1.0, 2.0)
            # rospy.loginfo(f"========== Uncertainty: {uncertainty:.6f} | Safety Distance: {self.safety_distance:.2f}")

        # Publish safety distance
        self.safety_distance_pub.publish(Float32(self.safety_distance))

        # Publish marker for visualization
        self.publish_safety_marker(object_position)

        # Compute and update velocity
        self.update_velocity(robot_position, object_position)

    def publish_safety_marker(self, object_position):
        """ Publishes a cylinder marker representing the safety distance around the object. """
        marker = Marker()
        marker.header.frame_id = "odom"  # Position is in the "odom" frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "safety_zone"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Set position
        marker.pose.position.x = object_position.x
        marker.pose.position.y = object_position.y
        marker.pose.position.z = 0.1  # Slightly above ground
        marker.pose.orientation.w = 1.0

        # Set scale (cylinder dimensions)
        marker.scale.x = self.safety_distance * 2  # Diameter
        marker.scale.y = self.safety_distance * 2  # Diameter
        marker.scale.z = 0.2  # Small height

        # Set color and transparency
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # Blue
        marker.color.a = 0.5  # Semi-transparent

        marker.lifetime = rospy.Duration(0.1)  # Short lifetime to refresh smoothly

        # Publish the marker
        self.marker_pub.publish(marker)

    def update_velocity(self, robot_position, object_position):
        """ Compute distance and dynamically adjust max_vel_x. """
        # Compute Euclidean distance
        dx = object_position.x - robot_position.x
        dy = object_position.y - robot_position.y
        distance = math.sqrt(dx**2 + dy**2)

        # Adjust velocity smoothly based on safety distance and distance
        if distance >= 3.0:
            desired_velocity = 1.2
        else:
            desired_velocity = 0.8
        
        # Update teb_local_planner max_vel_x dynamically
        if desired_velocity!=self.desired_velocity:
            self.desired_velocity = desired_velocity
            try:
                self.teb_client.update_configuration({"max_vel_x": desired_velocity})
                rospy.loginfo(f"Updated max_vel_x: {desired_velocity:.2f} m/s (Distance: {distance:.2f}m, Safety Distance: {self.safety_distance:.2f}m, Uncertainty: {np.trace(self.P):.2f})")
            except Exception as e:
                rospy.logwarn(f"Failed to update dynamic parameter: {e}")

if __name__ == '__main__':
    try:
        controller = VelocityController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
