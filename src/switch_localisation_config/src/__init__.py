#!/usr/bin/env python

import os 
import rospy 
import time

#os.system("export PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\u@\h:\w\$'")
#os.system("source ~/.bashrc")

def execute():
	rospy.init_node('execute',anonymous=True)
	time.sleep(10)
	#while not rospy.is_shutdown():
	os.system("cd ~/catkin_ws/bin/ && ./amcl_switcher")
	
	

if __name__=='__main__':
	try:
		execute()
	except rospy.ROSInterruptException:
		pass
