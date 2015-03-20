#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Header as HeaderMsg
from geometry_msgs.msg import PointStamped as PointStampedMsg
from geometry_msgs.msg import Pose2D as Pose2DMsg
from geometry_msgs.msg import PoseArray as PoseArrayMsg
from scenario_msgs.msg import Scenario as ScenarioMsg
from scenario_msgs.msg import Path as PathMsg
from scenario_msgs.msg import Obstacle as ObstacleMsg
from scenario_msgs.msg import ObstacleArray as ObstacleArrayMsg

class TopicRouter():
    def __init__(self):
        self.clickedPointSubscriber = rospy.Subscriber('/clicked_point', PointStampedMsg, self.clickedPointCB)
        self.clickedPointSubscriber = rospy.Subscriber('/robot01/path', PathMsg, self.pathCB)
        self.obstaclesPublisher = rospy.Publisher('/obstacles', ObstacleArrayMsg)
        self.scenarioPublisher = rospy.Publisher('/robot01/scenario', ScenarioMsg)
        self.pathVizPublisher = rospy.Publisher('/robot01/path_viz', PoseArrayMsg)
        
        self.obstacleArrayMsg = ObstacleArrayMsg()
        self.obstacleArrayMsg.header.frame_id = "/map"


    def pathCB(self, msg):
        self.pathVizPublisher.publish(msg.path.poses)
        
        
    def clickedPointCB(self, msg):
        if False:
            if msg.header.frame_id == "null":
                rospy.loginfo("Received remove clicked point")
                self.obstacleArrayMsg.obstacles = []
            else:
                rospy.loginfo("Received clicked point")
                obstacleMsg = ObstacleMsg(id = 54, x = msg.point.x, y = msg.point.y, radius = .3)
                self.obstacleArrayMsg.obstacles = [obstacleMsg]
            
            self.obstaclesPublisher.publish(self.obstacleArrayMsg)
        else:
            scenarioMsg = ScenarioMsg()
            headerMsg = HeaderMsg()
            scenarioMsg.type = "travel"
            headerMsg.frame_id = "/map"
            headerMsg.stamp = rospy.Time.now()
            scenarioMsg.bezier_paths.header = headerMsg
            scenarioMsg.target = Pose2DMsg()
            scenarioMsg.target.x = msg.point.x
            scenarioMsg.target.y = msg.point.y
            
            self.scenarioPublisher.publish(scenarioMsg)
            
        
                 
if __name__ == '__main__':
    # ros node
    rospy.init_node('topic_router', log_level = rospy.INFO)
    
    TopicRouter()
    
    rospy.spin()
