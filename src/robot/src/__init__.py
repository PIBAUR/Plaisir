#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os

import rospy

from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float64 as Float64Msg
from std_msgs.msg import Bool as BoolMsg
from scenario_msgs.msg import Scenario as ScenarioMsg

class RobotManager():
    INTERRUPTING_STATE = "INTERRUPTING_STATE"
    
    def __init__(self):
        self.statePublisher = rospy.Publisher('state', StringMsg)
        self.stopPathFollowerPublisher = rospy.Publisher("scenario", ScenarioMsg)
        self.stopPathFollowerMsg = ScenarioMsg()
        self.stopPathFollowerMsg.type = "stop"
        
        # states
        self.hasObstacle = False


    def freezeCB(self, msg):
        if msg.data:
            rospy.loginfo("Received freeze")
            self.stopPathFollowerPublisher.publish(self.stopPathFollowerMsg)


    def frontObstacleCB(self, msg):
        if msg.data and not self.hasObstacle:
            rospy.loginfo("Received new obstacle")
            self.stopPathFollowerPublisher.publish(self.stopPathFollowerMsg)
            # state
            #did that, but better that path_follower_choregraphic does that, because of latency 
            self.statePublisher.publish(StringMsg(RobotManager.INTERRUPTING_STATE))
            
        self.hasObstacle = msg.data
    
    
if __name__ == '__main__':
    robotManager = RobotManager()
    
    # ros node
    rospy.init_node("robot_manager", log_level = rospy.INFO)
    rospy.Subscriber("freeze", BoolMsg, robotManager.freezeCB)
    rospy.Subscriber("front_obstacle", BoolMsg, robotManager.frontObstacleCB)
    rospy.spin()
