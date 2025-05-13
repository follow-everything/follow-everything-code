#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped

class GoalPoseRepublisher:
    def __init__(self):
        rospy.init_node('goal_pose_republisher', anonymous=True)

        # Subscriber: listens to /goal_pose
        self.subscriber = rospy.Subscriber('/goal_pose', PoseStamped, self.callback)

        # Publisher: republishes to /move_base_simple/goal
        self.publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        rospy.loginfo("Goal Pose Republisher Node Started")

    def callback(self, msg):
        """ Callback function that republishes PoseStamped messages. """
        rospy.loginfo("Received goal, republishing to /move_base_simple/goal")
        self.publisher.publish(msg)

if __name__ == '__main__':
    try:
        GoalPoseRepublisher()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
