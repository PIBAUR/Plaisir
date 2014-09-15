#!/usr/bin/env python
import roslib
import rospy
from math import sin, cos, atan2

from geometry_msgs.msg import PointStamped
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from math import sqrt
from math import asin
from test_suivi.srv import *

class NeatoPublishPose :
    def __init__(self):
        rospy.init_node('neato_publish_pose')
        rospy.Subscriber("/point_sent", PointStamped, self.PtStpCb)
        s = rospy.Service('send_pose', SendPose, self.handle_send_pose)
        self.rosloop = rospy.Rate(10)
        self.posestp = [ ]
        self.base_link_listener =  tf.TransformListener()
        self.robot_pos = []
        self.robot_quat = []
        self.robot_posx = 0.0
        self.robot_posy = 0.0
        self.i = 0

        
    
    def PtStpCb(self,req):
        #self.vel_des = [ req.linear.x , req.angular.z ]

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = req.point.x
        pose_stamped.pose.position.y = req.point.y
        pose_stamped.pose.position.z = 0

        qz = 0
        qw = 1

        if len(self.posestp) > 0 :
            dx = req.point.x - self.posestp[-1].pose.position.x
            dy = req.point.y - self.posestp[-1].pose.position.y
            th = atan2(dy,dx)
            qz = sin(th/2.0)
            qw = cos(th/2.0)
        else :
            (self.robot_pos ,self.robot_quat) = self.base_link_listener.lookupTransform('/odom', '/base_laser_link', rospy.Time(0))
            self.robot_posx = self.robot_pos[0]
            self.robot_posy = self.robot_pos[1]
            dx = req.point.x - self.robot_posx
            dy = req.point.y - self.robot_posy
            th = atan2(dy,dx)
            qz = sin(th/2.0)
            qw = cos(th/2.0)

        pose_stamped.pose.orientation.x = 0
        pose_stamped.pose.orientation.y = 0
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw
        pose_stamped.header.frame_id = req.header.frame_id
        pose_stamped.header.stamp = req.header.stamp
        
        self.posestp.append(pose_stamped)
        
        
        
    def handle_send_pose (self, req):
    #j'avoue c'est moche
        
        print ("Pose desired : {}".format(self.posestp[self.i]))
        self.i = self.i+1
        return SendPoseResponse(self.posestp[self.i - 1])  #j'avoue c'est tres moche
        
        
    def spin(self):
        while not rospy.is_shutdown():
            self.rosloop.sleep()
        
                
                
        
if __name__ == "__main__":
    neatopublishpose = NeatoPublishPose()
    neatopublishpose.spin()
