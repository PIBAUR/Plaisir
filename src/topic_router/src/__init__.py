#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Header as HeaderMsg
from geometry_msgs.msg import PointStamped as PointStampedMsg
from geometry_msgs.msg import Pose2D as Pose2DMsg
from geometry_msgs.msg import PoseArray as PoseArrayMsg
from scenario_msgs.msg import Scenario as ScenarioMsg
from scenario_msgs.msg import PathPosition as PathPositionMsg
from scenario_msgs.msg import Obstacle as ObstacleMsg
from scenario_msgs.msg import ObstacleArray as ObstacleArrayMsg

class TopicRouter():
    def __init__(self):
        self.clickedPointSubscriber = rospy.Subscriber('/clicked_point', PointStampedMsg, self.clickedPointCB)
        self.pathTravelSubscriber = rospy.Subscriber('path_travel', PathPositionMsg, self.pathCB)
        self.pathChoregraphicSubscriber = rospy.Subscriber('path_choregraphic', PathPositionMsg, self.pathCB)
        self.obstaclesPublisher = rospy.Publisher('/obstacles', ObstacleArrayMsg)
        
        self.pathVizPublisher = rospy.Publisher('path_viz', PoseArrayMsg)

    def pathCB(self, msg):
        self.pathVizPublisher.publish(PoseArrayMsg(header = HeaderMsg(frame_id = "/map"), poses = msg.path.poses))
        #TODO: remove some pose in path if size(path) > SIZE_MAX
        """
        size_array = len(msg.path.poses)
        if size_array<250 :
            self.pathVizPublisher.publish(PoseArrayMsg(header = HeaderMsg(frame_id = "/map"), poses = msg.path.poses))
        else:
            poseArray = PoseArrayMsg()
            for i in range(250):
                index = int(round(float(i*size_array)/250.0))
                if index < 250 :
                    poseArray.poses.append(msg.path.poses[index])
            
            self.pathVizPublisher.publish(PoseArrayMsg(header = HeaderMsg(frame_id = "/map"), poses = poseArray.poses))
        """
        
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
