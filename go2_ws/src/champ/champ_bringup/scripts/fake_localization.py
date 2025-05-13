#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped

class OdomTransformNode(Node):
    def __init__(self):
        super().__init__('odom_transform_node')
        
        # Subscriber to the original odometry topic
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/ground_truth',
            self.odom_callback,
            30
        )
        
        # Publisher for the modified odometry topic
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            30
        )
        
        # TF2 broadcaster for the transform from odom to base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
    def odom_callback(self, msg):
        # Modify the frame_id from "world" to "odom"
        new_odom = msg
        new_odom.header.frame_id = "odom"
        
        # Publish the modified odometry message
        self.odom_pub.publish(new_odom)
        
        # Publish the transform from odom to base_link
        self.publish_tf_transform(msg)
        
    def publish_tf_transform(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        # Copy the position and orientation from the odometry message
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t)
        

def main(args=None):
    rclpy.init(args=args)
    node = OdomTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
