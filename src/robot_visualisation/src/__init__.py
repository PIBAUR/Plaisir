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
    frame_id="/"
    if rospy.get_param("tf_prefix"):
        prefix=rospy.get_param("tf_prefix")
        frame_id+=prefix
        frame_id+="/base_link"
        print(frame_id) 
    
    try:
        trans, rot = listener.lookupTransform('/map',frame_id, rospy.Time(0))
        angles = euler_from_quaternion(rot)
        yaw = angles[2]
    except Exception, e:
        rospy.logerr(e)

    position=(trans[0],trans[1],yaw)
    
    if prefix=="robot01":
 
        os.system("roslaunch robot_visualisation delete_robot.launch robot:=robot0")
        os.system("roslaunch robot_visualisation add_robot.launch position:="+"\"" + " ".join([str(item) for item in position[0:3]])+"\"") 
           
    else:
        os.system("roslaunch robot_visualisation add_robot.launch position:="+"\"" + " ".join([str(item) for item in position[0:3]])+"\"")
    
        
        
if __name__=='__main__':
    try:
        execute()
    except rospy.ROSInterruptException:
        pass
