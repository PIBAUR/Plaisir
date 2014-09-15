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

class NeatoPublishCommand:
	def __init__(self):
		rospy.init_node('neato_publish_command')
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
		self.rosloop = rospy.Rate(10)
		self.cmd_vel = [0.0,0.0]
		self.olddu = 0.0
		self.k = 0


	def getting_current_pos(self):
		maj_pose = True
		while not rospy.is_shutdown() and maj_pose == True:
			try:

				(self.robot_pos ,self.robot_quat) = self.base_link_listener.lookupTransform('/odom', '/base_laser_link', rospy.Time(0))
				self.robot_posx = self.robot_pos[0]
				self.robot_posy = self.robot_pos[1]
				(roll, pitch, yaw ) = euler_from_quaternion(self.robot_quat)
				self.robot_posth = yaw
				maj_pose = False
				#print("Current pose : {} and current orientation :{}".format(self.robot_pos, self.robot_posth))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			



	def send_command(self):
		twist = Twist()
		dx = self.robot_posx_goal - self.robot_posx
		dy = self.robot_posy_goal - self.robot_posy
		dth = self.robot_posth_goal - self.robot_posth
		du = sqrt(dx*dx+dy*dy)
		
		#print ("du = {} dth = {}".format(du, dth))
		
		if (du <=0.5  and abs(dth) <=0.05):
			print("robot pose : {}  ,  {}    point : {}  ,  {}".format(self.robot_posx,self.robot_posy,self.robot_posx_goal,self.robot_posy_goal))
			print("erreur : {}  ,  {} ".format(self.robot_posx_goal - self.robot_posx, self.robot_posy_goal - self.robot_posy))
			self.k = 0
			self.next_pose = True

		else :
			##if (du  > 0.5 and abs(dth)> 0.05): #tests avec du = 1 et du = 0.5; dth = 0.05 et dth = 0.1
			##	kd = 0.05/2.0
			##	kth=0.5
			##	if not dy == 0 and not dx == 0:
			##		twist.linear.x = 0.3
			##	else :
			##		if dx == 0:
			##			twist.linear.x = 0.3
			##		if dy == 0:
			##			twist.linear.x = 0.3
			##	twist.angular.z = kth*dth
			##	print("1111111111111111111111111111111111")
            ##
			##if (du > 0.5 and abs(dth) <0.05 ): 
			##	kd = 0.2/5.0
			##	kth = 0.1
			##	if not dy == 0 and not dx == 0:
			##		twist.linear.x = 0.3
			##	else :
			##		if dx == 0:
			##			twist.linear.x = 0.3
			##		if dy == 0:
			##			twist.linear.x =0.3
			##	twist.angular.z = kth*dth*10
			##	print("222222222222222222222222222222222")
            ##
			##if (du < 0.5 and abs(dth) > 0.05): 
			##	kd = 0.1/5.0
			##	kth = 0.35
			##	if not dy == 0 and not dx == 0:
			##		twist.linear.x = 0.3       
			##	else :
			##		if dx == 0:
			##			twist.linear.x = 0.3
			##		if dy == 0:
			##			twist.linear.x = 0.3
			##	twist.angular.z = kth*dth*10
			##	print("333333333333333333333333333333333")
			##self.next_pose = False

			self.next_pose = False
		twist.angular.z = sin(dth)*2.0
		twist.linear.x = cos(abs(dth))*0.05

		self.k = self.k +1
		#print ("Twist : {}\t{}".format(twist.linear.x, twist.angular.z))
		self.commandPub.publish(twist)
		


	def request_next_pose(self):
		rospy.wait_for_service('send_pose')
		try:
			pose = rospy.ServiceProxy('send_pose', SendPose)
			resp = pose()
			self.robot_posx_goal = resp.p.pose.position.x
			self.robot_posy_goal = resp.p.pose.position.y
			quaternion = (resp.p.pose.orientation.x, resp.p.pose.orientation.y, resp.p.pose.orientation.z, resp.p.pose.orientation.w)
			(rollg, pitchg, yawg) = euler_from_quaternion(quaternion)
			self.robot_posth_goal = yawg
			self.next_pose = False
		except rospy.ServiceException, e:
			print "Service call failed :%s"%e		
				

	def spin (self):
		while not rospy.is_shutdown():
			if self.next_pose == True:
				self.request_next_pose()
			self.getting_current_pos()
			#print("Calculating for command")
			self.send_command()
			#print("Sending command")
			self.rosloop.sleep()

if __name__ == "__main__":
    neatopublishcommand = NeatoPublishCommand()
    neatopublishcommand.spin()
