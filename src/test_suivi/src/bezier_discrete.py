#!/usr/bin/env python
import roslib
import rospy
from math import sin, cos, atan2

from geometry_msgs.msg import PointStamped
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from math import sqrt
from math import asin


class BezierDiscrete:
    def __init__(self):
        rospy.init_node('bezier_discrete')
        self.pointPub = rospy.Publisher ("/point_sent", PointStamped)
        self.t = 0.0
        self.rosloop = rospy.Rate(10)        
        self.robot_pos = []
        self.robot_quat = []
        self.base_link_listener =  tf.TransformListener()
        

    def send_point(self):
		ptstp = PointStamped()
		
		ptstp.header.frame_id = 'odom'
#        ptstp.point.x = sin(self.t/10)/(1+cos(self.t/10)*cos(self.t/10))*5
#        ptstp.point.y = sin(self.t/10)*cos(self.t/10)/(1+cos(self.t/10)*cos(self.t/10))*5
#        ptstp.point.z = 0
		ptstp.point.x = self.t/10.0
		ptstp.point.y = (-1+cos(5.0*self.t/10.0))/2.0
		self.pointPub.publish(ptstp)
        #print ("x : {} and y : {}".format(ptstp.point.x, ptstp.point.y))
		self.t = self.t + 1
        
    def spin(self):
        while not rospy.is_shutdown():
            self.send_point()
            self.rosloop.sleep()
            
if __name__ == "__main__":
    bezierdiscrete = BezierDiscrete()
    bezierdiscrete.spin()
