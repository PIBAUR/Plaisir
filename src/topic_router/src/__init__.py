#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import PointStamped as PointStampedMsg
from scenario_msgs.msg import Obstacle as ObstacleMsg
from scenario_msgs.msg import ObstacleArray as ObstacleArrayMsg

class TopicRouter():
    def __init__(self):
        self.clickedPointSubscriber = rospy.Subscriber('/clicked_point', PointStampedMsg, self.clickedPointCB)
        self.obstaclesPublisher = rospy.Publisher('/obstacles', ObstacleArrayMsg)
        
        self.obstacleArrayMsg = ObstacleArrayMsg()
        self.obstacleArrayMsg.header.frame_id = "/map"


    def clickedPointCB(self, msg):
        if msg.header.frame_id == "null":
            rospy.loginfo("Received remove clicked point")
            self.obstacleArrayMsg.obstacles = []
        else:
            rospy.loginfo("Received clicked point")
            obstacleMsg = ObstacleMsg(id = 54, x = msg.point.x, y = msg.point.y, radius = .3)
            self.obstacleArrayMsg.obstacles = [obstacleMsg]
        
        self.obstaclesPublisher.publish(self.obstacleArrayMsg)
        
                 
if __name__ == '__main__':
    # ros node
    rospy.init_node('topic_router', log_level = rospy.INFO)
    
    TopicRouter()
    
    rospy.spin()
