#!/usr/bin/env python

import os 
import roslib
import rospy 
import tf 
from tf.transformations import euler_from_quaternion

os.system("export PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\u@\h:\w\$'")
os.system("source ~/.bashrc")

def execute():
	rospy.init_node('execute',anonymous=True)
	listener = tf.TransformListener()
	rate=rospy.Rate(10.0) 
	rate.sleep()
	#while not rospy.is_shutdown():
	try:
		trans, rot = listener.lookupTransform('/map','/robot01/base_link', rospy.Time(0))
		angles = euler_from_quaternion(rot)
		yaw = angles[2]
	except Exception, e:
		rospy.logerr(e)

	position=(trans[0],trans[1],yaw)
	#print("roslaunch robot test.launch position:="+"\"" + " ".join([str(item) for item in test[0:3]])+"\"")
	os.system("roslaunch robot_visualisation visualisation_environment.launch position:="+"\"" + " ".join([str(item) for item in position[0:3]])+"\"")
	#rate.sleep()
	

if __name__=='__main__':
	try:
		execute()
	except rospy.ROSInterruptException:
		pass
