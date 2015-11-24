#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import tf
from scenario_msgs.msg import PathTravel as PathTravelMsg
from scenario_msgs.msg import PathChoregraphic as PathChoregraphicMsg
from scenario_msgs.msg import TimeAtPose as TimeAtPoseMsg
from scenario_msgs.msg import TimeAtPoseArray as TimeAtPoseArrayMsg
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import TwistStamped as TwistStampedMsg
from geometry_msgs.msg import Point as PointMsg
from std_msgs.msg import Float64 as Float64Msg
from geometry_msgs.msg import Quaternion
from scenario_msgs.msg import TwistStampedArray as TwistStampedArrayMsg


def pathCallback(msg):
    rospy.loginfo("Receive path with " + str(len(msg.path.poses)) + " poses, start at " + str(msg.start_timestamp)\
                   + " with " + str(len(msg.time_at_poses.time_at_poses)) )
    pathMsg = createPathChoregraphic(msg)
    #rospy.loginfo(str(pathMsg))
    pathChoregraphicPublisher.publish(pathMsg)


def getSequenceLength(path, start, end):
    length = 0.0
    
    for i in range(start, end-1):
        dx = path.poses[i].position.x -path.poses[i+1].position.x
        dy = path.poses[i].position.y -path.poses[i+1].position.y
        length += math.sqrt(dx*dx+dy*dy)
   
    return length


def getSequenceTime(timeAtPoses, start, end):
    return timeAtPoses[end].time - timeAtPoses[start].time

def getAngle(pose1, pose2):
    q1 = (pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w)
    q2 = (pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w)
    e1 = tf.transformations.euler_from_quaternion(q1)
    e2 = tf.transformations.euler_from_quaternion(q2)
    th1 = e1[2]
    th2 = e2[2]
    return (th2-th1)

def getDistance(pose1, pose2):
    dx = pose2.position.x - pose1.position.x
    dy = pose2.position.y - pose1.position.y
    return math.sqrt(dx*dx+dy*dy)

def createPathChoregraphic(msg):
    pathMsg = PathChoregraphicMsg()
    sequences = msg.time_at_poses.time_at_poses
    
    path = msg.path
    
    startTime = rospy.Time.now()# + rospy.Duration.from_sec(10.0)
    #TODO : Uncomment after update of "scenario publisher"
    #startTime.secs = msg.start_timestamp.secs
    #startTime.nsecs = msg.start_timestamp.nsecs
    #pathMsg.start_timestamp = startTime
    pathMsg.start_timestamp.secs = startTime.secs
    pathMsg.start_timestamp.nsecs = startTime.nsecs
    
    rospy.loginfo(str(startTime))
    nb_point = 0
    total_time = 0.0
    
    for s in range(len(sequences)-1):
        seq = {}
        if sequences[s].backward == 1:
            seq["backward"] = -1.0
        else :
            seq["backward"] = 1.0
        seq["start time"] = startTime + rospy.Duration(sequences[s].time) 
        seq["duration"] = getSequenceTime(sequences, s, s+1)
        seq["nb poses"] = sequences[s+1].pose_index - sequences[s].pose_index +1
        seq["time step"] = seq["duration"]/seq["nb poses"]
        seq["distance"] = getSequenceLength(path, sequences[s].pose_index, sequences[s+1].pose_index)
        total_time += seq["duration"] 
        rospy.loginfo("Sequence #" + str(s) + "/" + str(len(sequences)-1) +\
                      " : start time = " + str(seq["start time"].secs) + " duration = " + str(seq["duration"]) +\
                      " nb poses = " + str(seq["nb poses"]) + " time step = " + str(seq["time step"]) +\
                      " distance = " + str(seq["distance"]) )
        
        if seq["nb poses"] == 1 :
            ts = TwistStampedMsg()
            #header
            ts.header.frame_id = path.header.frame_id
            ts.header.stamp.nsecs =  (seq["start time"] + rospy.Duration(seq["time step"])).nsecs
            ts.header.stamp.secs =  (seq["start time"] + rospy.Duration(seq["time step"])).secs
            #twist
            ts.twist.angular.x = 0.0
            ts.twist.angular.y = 0.0
            ts.twist.angular.z = 0.0
            ts.twist.linear.x = 0.0
            ts.twist.linear.y = 0.0
            ts.twist.linear.z = 0.0
            pathMsg.path.twists.append(ts)
            nb_point += 1
        else :
            for i in range(seq["nb poses"]-2):
                ts = TwistStampedMsg()
                #header
                ts.header.frame_id = path.header.frame_id
                ts.header.stamp.nsecs =  (seq["start time"] + rospy.Duration(seq["time step"] * i)).nsecs
                ts.header.stamp.secs =  (seq["start time"] + rospy.Duration(seq["time step"] * i)).secs
                #ts.header.stamp.secs =  rospy.Time(seq["start time"] + rospy.Duration(seq["time step"] * i)).secs
                #ts.header.stamp.nsecs =  rospy.Time(seq["start time"] + seq["time step"] * i).nsecs
                #twist
                ts.twist.angular.x = 0.0
                ts.twist.angular.y = 0.0
                ts.twist.angular.z = getAngle(path.poses[sequences[s].pose_index + i], path.poses[sequences[s].pose_index+i+1]) / seq["time step"]
                ts.twist.linear.x = seq["backward"]*getDistance(path.poses[sequences[s].pose_index + i], path.poses[sequences[s].pose_index+i+1])/seq["time step"]
                ts.twist.linear.y = 0.0
                ts.twist.linear.z = 0.0
                pathMsg.path.twists.append(ts)
                nb_point += 1
    
    rospy.loginfo("Create path with " + str(nb_point) + " poses, start at " + str(pathMsg.start_timestamp.secs)) 
    return pathMsg




if __name__ == "__main__":
    rospy.init_node('pose_to_twist', anonymous = False)
    
    rospy.Subscriber("path_choregraphic", PathTravelMsg, pathCallback)
    
    pathChoregraphicPublisher = rospy.Publisher("twist_path", PathChoregraphicMsg)
    
    rospy.spin()