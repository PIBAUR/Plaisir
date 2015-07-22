#!/usr/bin/env python

import os 
import rospy 
import time

#os.system("export PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\u@\h:\w\$'")
#os.system("source ~/.bashrc")

def execute():
    rospy.init_node('execute',anonymous=True)
    time.sleep(10)
    if rospy.get_param("tf_prefix2"):
        prefix=rospy.get_param("tf_prefix2")
    
    os.system("roslaunch switch_localisation_config run.launch robot:="+prefix[-2]+prefix[-1])
    
    

if __name__=='__main__':
    try:
        execute()
    except rospy.ROSInterruptException:
        pass
