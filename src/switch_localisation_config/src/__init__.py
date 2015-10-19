#!/usr/bin/env python

import os 
import rospy 
import time
import tf
import sys

#os.system("export PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\u@\h:\w\$'")
#os.system("source ~/.bashrc")

def execute():
    rospy.init_node('execute',anonymous=True)
    rospy.loginfo("Waiting for init 20 sec...")
    time.sleep(15)
    
    baseFrame = "/base_link"
    mapFrame = "/map"
    xRobot = 0.0
    yRobot = 0.0
    thRobot = 0.0

    if rospy.get_param("tf_prefix"):
        tfPrefix=rospy.get_param("tf_prefix")
        baseFrame = "/"+str(tfPrefix)+str(baseFrame)
    
    tfListener = tf.TransformListener()
    
    rospy.loginfo("tf_prefix : " + str(tfPrefix))
    
    rospy.sleep(5)
    
    try:
        (trans,rot) = tfListener.lookupTransform(str(mapFrame), str(baseFrame), rospy.Time(0))
        rospy.loginfo("Get tf")
        xRobot = trans[0]
        yRobot = trans[1]
        rospy.loginfo("set pose")
        thRobot =  tf.transformations.euler_from_quaternion(rot)[-1]
        rospy.loginfo("set yaw")
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Unable to get tf between %s and %s",mapFrame, baseFrame) 
        pass

    xArg = " robot_switch_x:=" + str(xRobot)
    yArg = " robot_switch_y:=" + str(yRobot)
    thArg = " robot_switch_a:=" + str(thRobot)
    
    os.system("rosnode kill /" + str(tfPrefix) + "/amcl")
    rospy.loginfo("Killing node "+ str(tfPrefix) + "/amcl")
    rospy.sleep(1)
    os.system("roslaunch robot server_localisation_bis.launch init_done:=true robot:="+str(tfPrefix[-2:])+xArg+yArg+thArg)
    #sys.exit():
    

if __name__=='__main__':
    execute()
    while not rospy.is_shutdown() :
        rospy.sleep(5)
