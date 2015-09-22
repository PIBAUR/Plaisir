#!/usr/bin/env python

import os 
import roslib
import rospy 
import tf 
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid as NavMsg

os.system("export PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\u@\h:\w\$'")
os.system("source ~/.bashrc")

map_pose = NavMsg()

def callback(data):
    #try:
        #rospy.Subscriber("/robot01/amcl_pose_bis", GeometryMsg, map_callback)
    map_pose.info.resolution=data.info.resolution
    map_pose.info.origin.position=data.info.origin.position
    #map_pose.info.origin.position.y=data.info.origin.position.y
    map_pose.info.origin.orientation=data.info.origin.orientation

def execute():
    rospy.init_node('execute',anonymous=True)
    listener = tf.TransformListener()

    #rate=rospy.Rate(7) 
    rate=rospy.Rate(7) 
    try:
        rospy.Subscriber("/map", NavMsg, callback)
    except Exception, e:
        rospy.logerr(e)
        
    rate.sleep()
    #while not rospy.is_shutdown():
    frame_id=""
    if rospy.get_param("tf_prefix"):
        prefix=rospy.get_param("tf_prefix")
        frame_id+=prefix
        frame_id+="/base_link"
        print(frame_id) 
        
    #rospy.loginfo(rospy.get_caller_id() + "I heard %f", map_pose.info.resolution)
 
    
    try:
        listener.waitForTransform("/map", frame_id, rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("/map",frame_id, rospy.Time(0))
        angles = euler_from_quaternion(rot)
        yaw = angles[2]
    except Exception, e:
        rospy.logerr(e)
    
    
    map_angles=(map_pose.info.origin.orientation.x,map_pose.info.origin.orientation.y,map_pose.info.origin.orientation.z,map_pose.info.origin.orientation.w)
    map_yaw=euler_from_quaternion(map_angles)

    position=(trans[0]-map_pose.info.origin.position.x,trans[1]-map_pose.info.origin.position.y,yaw +map_yaw[2])
    print(position) 
    if prefix=="robot01":
        #os.system("roslaunch robot_visualisation delete_robot.launch robot:=robot0")
        os.system("roslaunch robot_visualisation visualisation_environment.launch position:="+"\"" + " ".join([str(item) for item in position[0:3]])+"\"") 
                   
    
    if prefix!="robot01" :
        if prefix=="robot02" :
            os.system("roslaunch robot_visualisation add_robot.launch position:="+"\"" + " ".join([str(item) for item in position[0:3]])+"\"")
        else:
            os.system("roslaunch robot_visualisation delete_robot.launch position:="+"\"" + " ".join([str(item) for item in position[0:3]])+"\"")
        
        
if __name__=='__main__':
    try:
        execute()
    except rospy.ROSInterruptException:
        pass
