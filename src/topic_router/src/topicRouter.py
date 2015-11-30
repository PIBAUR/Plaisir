#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math

import rospy
import tf

from std_msgs.msg import Header as HeaderMsg
from geometry_msgs.msg import PointStamped as PointStampedMsg
from geometry_msgs.msg import PoseArray as PoseArrayMsg
from geometry_msgs.msg import Pose as PoseMsg
from scenario_msgs.msg import Scenario as ScenarioMsg
from scenario_msgs.msg import Obstacle as ObstacleMsg
from scenario_msgs.msg import ObstacleArray as ObstacleArrayMsg

class TopicRouter():
    def __init__(self):
        routeType = None
        routeTopicFrom = None
        routeTopicTo = None
        for arg in sys.argv:
            if arg.startswith("route_type:="):
                routeType = arg.replace("route_type:=", "")
            if arg.startswith("route_topic_from:="):
                routeTopicFrom = arg.replace("route_topic_from:=", "")
            if arg.startswith("route_topic_to:="):
                routeTopicTo = arg.replace("route_topic_to:=", "")
        
        if routeType == "PointStampedMsg_to_ObstacleArrayMsg":
            self.clickedPointSubscriber = rospy.Subscriber(routeTopicFrom, PointStampedMsg, self.clickedPointCB)
            self.obstaclesPublisher = rospy.Publisher(routeTopicTo, ObstacleArrayMsg)
        elif routeType == "ScenarioMsg_to_PoseArrayMsg":
            self.scenarioSubscriber = rospy.Subscriber(routeTopicFrom, ScenarioMsg, self.scenarioCB)
            self.pathVizPublisher = rospy.Publisher(routeTopicTo, PoseArrayMsg)


    def scenarioCB(self, msg):
        result = PoseArrayMsg(header = HeaderMsg(frame_id = "/map"))
        if len(msg.checkedChoregraphicPath.poses) > 0:
            result = msg.checkedChoregraphicPath.poses
        else:
            result.poses = []
            for curve in msg.bezier_paths.curves:
                pose = PoseMsg()
                pose.position.x = curve.anchor_1.x
                pose.position.y = curve.anchor_1.y
                orientation = math.atan2(curve.control_1.y - curve.anchor_1.y, curve.control_1.x - curve.anchor_1.x)
                quaternion = tf.transformations.quaternion_from_euler(0, 0, orientation)
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
                result.poses.append(pose)
        
        self.pathVizPublisher.publish(result)

        
    def clickedPointCB(self, msg):
        result = ObstacleArrayMsg()
        result.header.frame_id = "/map"
        
        if msg.header.frame_id == "null":
            rospy.loginfo("Received remove clicked point")
        else:
            rospy.loginfo("Received clicked point: x=" + str(msg.point.x) + "; y=" + str(msg.point.y))
            obstacleMsg = ObstacleMsg(id = 54, x = msg.point.x, y = msg.point.y, radius = .3)
            result.header.frame_id = "/map"
            result.obstacles = [obstacleMsg]
        
        self.obstaclesPublisher.publish(result)