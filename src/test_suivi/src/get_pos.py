#!/usr/bin/env python
import roslib; roslib.load_manifest("neato_node")
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

class GetPos:
	def __init__(self):
		rospy.init_node('get_pos')
		self.commandPub = rospy.Publisher ("cmd_vel", Twist)
		self.base_link_listener =  tf.TransformListener()
		self.robot_pos = []
		self.robot_quat = []
		self.robot_posx = 0.0
		self.robot_posx_goal = 0.0		
		self.robot_posy = 0.0
		self.robot_posy_goal = 0.0
		self.robot_posth = 0.0
		self.robot_posth_goal = 0.0
		self.next_pose = True
		self.rosloop = rospy.Rate(1)
		self.cmd_vel = [0.0,0.0]


	def getting_current_pos(self):
		while not rospy.is_shutdown():
			try:

				(self.robot_pos ,self.robot_quat) = self.base_link_listener.lookupTransform('/odom', '/base_laser_link', rospy.Time(0))
				self.robot_posx = self.robot_pos[0]
				self.robot_posy = self.robot_pos[1]
				(roll, pitch, yaw ) = euler_from_quaternion(self.robot_quat)
				self.robot_posth = yaw
				
				print("Position : {}  Orientation :{}".format(self.robot_pos, self.robot_posth))
				self.rosloop.sleep()
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue


	def spin (self):
		while not rospy.is_shutdown():
			self.getting_current_pos()

if __name__ == "__main__":
    neatopublishcommand = GetPos()
    neatopublishcommand.spin()
