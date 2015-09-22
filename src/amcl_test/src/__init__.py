#!/usr/bin/env python

""" --------------------------------------------------------------------
    TO CHANGE DEPENDING ON THE PACKAGE:
    DON'T FORGET TO CREATE AN launch/eclipse.launch WITH THE CORRECT PACKAGE
"""
NODE_NAME = "amcl_test"
""" -------------------------------------------------------------------- """

import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from std_msgs.msg import Bool

rospy.init_node('amcl_init_pose', anonymous = True)
#pub_pose = rospy.Publisher('/robot01/initialpose_bis',PoseWithCovarianceStamped)
pub_pose = rospy.Publisher('/initialpose_bis',PoseWithCovarianceStamped)
initialpose_msg = PoseWithCovarianceStamped()
initialpose_msg.header.frame_id = "/map"
initialpose_msg.pose.pose.position.x =1
initialpose_msg.pose.pose.position.y =1
initialpose_msg.pose.pose.position.z =0
initialpose_msg.pose.pose.orientation.x =0
initialpose_msg.pose.pose.orientation.y =0
initialpose_msg.pose.pose.orientation.z =0
initialpose_msg.pose.pose.orientation.w = 1
    


def poseCallback(msg):
    rospy.loginfo("in CB")
    #print("in CB")
    initialpose_msg.pose = PoseWithCovariance()
    initialpose_msg.pose.pose = msg.pose.pose

def publishTriggerCallback(msg):
    
    rospy.loginfo(initialpose_msg)
    print("Trigger")
    rospy.loginfo(initialpose_msg.header)
    rospy.loginfo(initialpose_msg.pose)
    rospy.loginfo(initialpose_msg.pose.pose)
    rospy.loginfo(initialpose_msg.pose.pose.position)
    rospy.loginfo(initialpose_msg.pose.pose.orientation)
    
    pub_pose.publish(initialpose_msg)
    
if __name__ == '__main__':
    
    #print("ok")
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, poseCallback)
    #rospy.Subscriber("/robot01/publish_trigger", Bool, publishTriggerCallback)
    rospy.Subscriber("/publish_trigger", Bool, publishTriggerCallback)
    
    
    while not rospy.is_shutdown():
            rospy.spin()
