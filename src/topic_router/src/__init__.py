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
        self.clickedPointSubscriber = rospy.Subscriber('path', PathMsg, self.pathCB)
        self.obstaclesPublisher = rospy.Publisher('/obstacles', ObstacleArrayMsg)
        
        self.pathVizPublisher = rospy.Publisher('path_viz', PoseArrayMsg)

    def pathCB(self, msg):
        self.pathVizPublisher.publish(PoseArrayMsg(header = HeaderMsg(frame_id = "/map"), poses = msg.path.poses))
        
        
    def clickedPointCB(self, msg):
        obstacleArrayMsg = ObstacleArrayMsg()
        obstacleArrayMsg.header.frame_id = "/map"
        
        if msg.header.frame_id == "null":
            rospy.loginfo("Received remove clicked point")
        else:
            rospy.loginfo("Received clicked point")
            obstacleMsg = ObstacleMsg(id = 54, x = msg.point.x, y = msg.point.y, radius = .3)
            obstacleArrayMsg.header.frame_id = "/map"
            obstacleArrayMsg.obstacles = [obstacleMsg]
        
        rospy.loginfo(obstacleArrayMsg)
        self.obstaclesPublisher.publish(obstacleArrayMsg)
            
        
                 
if __name__ == '__main__':
    # ros node
    rospy.init_node('topic_router', log_level = rospy.INFO)
    
    TopicRouter()
    
    rospy.spin()
