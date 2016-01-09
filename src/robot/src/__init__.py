#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import threading
import time
import random
import math

import rospy

from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float64 as Float64Msg
from std_msgs.msg import Bool as BoolMsg
from scenario_msgs.msg import Scenario as ScenarioMsg
from geometry_msgs.msg import Twist as TwistMsg
from geometry_msgs.msg import Vector3 as Vector3Msg




K_LINEAR = rospy.get_param("robot_manager_k_linear", 0.5)
KL_ANGULAR = rospy.get_param("robot_manager_kl_angular", 2.0)
KA_ANGULAR = rospy.get_param("robot_manager_ka_angular", 1.0)
rospy.loginfo("Coefficient for K_LINEAR, KL_ANGULAR, KA_ANGULAR : " + 
              str(K_LINEAR) + "\t" + str(KL_ANGULAR) + "\t" + str(KA_ANGULAR) + "\t")

class TurnThread(threading.Thread):
    def __init__(self):
        super(TurnThread, self).__init__()
        
        self.cmdVelPublisher = rospy.Publisher("cmd_vel", TwistMsg)
        self.turnMsg = TwistMsg(Vector3Msg(0, 0, 0), Vector3Msg(0, 0, 0))
        self.turnState = False
        self.turnLeft = False
        
        self.currentCmdVel = None
        self.kill = False
        
        
    def run(self):
        while True:
            if self.turnState:
                if self.currentCmdVel is not None:
                    self.turnMsg.angular.z = (1 if self.turnLeft else -1) * (abs(self.currentCmdVel.linear.x) * KL_ANGULAR +
                                                                              abs(self.currentCmdVel.angular.z) * KA_ANGULAR)
                    """
                    Add linear.x so the robot can move backward while turning
                    """
                    self.turnMsg.linear.x = (-abs(self.currentCmdVel.linear.x) * K_LINEAR)
                else:
                    self.turnMsg.angular.z = .2
                    
                self.cmdVelPublisher.publish(self.turnMsg)
            
            if self.kill:
                break
            time.sleep(.01)
    
    
    def turn(self, turnLeft):
        rospy.loginfo("start turn")
        self.turnState = True
        self.turnLeft = turnLeft
    
    
    def stopTurn(self):
        rospy.loginfo("stop turn")
        self.turnState = False
        self.turnMsg.angular.z = 0
        self.cmdVelPublisher.publish(self.turnMsg)
        
    
    def setCurrentCmdVel(self, msg):
        self.currentCmdVel = msg
        

class RobotManager():
    INTERRUPTING_STATE = "INTERRUPTING_STATE"
    
    def __init__(self):
        self.statePublisher = rospy.Publisher("state", StringMsg)
        self.stopPathFollowerPublisher = rospy.Publisher("scenario", ScenarioMsg)
        self.freezePathFollowerPublisher = rospy.Publisher("freeze_path_follower", BoolMsg)
        self.stopPathFollowerMsg = ScenarioMsg()
        self.stopPathFollowerMsg.type = "stop"
        
        self.turnThread = TurnThread()
        self.turnThread.start()
        
        # states
        self.hasObstacle = False


    def wantedCmdVelCB(self, msg):
        self.turnThread.setCurrentCmdVel(msg)
        
        
    def freezeCB(self, msg):
        if msg.data:
            rospy.loginfo("Received freeze")
            self.stopPathFollowerPublisher.publish(self.stopPathFollowerMsg)


    def frontObstacleCB(self, msg):
        if msg.data and not self.hasObstacle:
            rospy.loginfo("Received new obstacle")
            self.freezePathFollowerPublisher.publish(BoolMsg(True))
            self.turnThread.turn(random.random() > .5) # <----- Holy shit! That's smart! Nice found! 
            #self.stopPathFollowerPublisher.publish(self.stopPathFollowerMsg)
            # state
            #did that, but better that path_follower_choregraphic does that, because of latency 
            #self.statePublisher.publish(StringMsg(RobotManager.INTERRUPTING_STATE))
        elif not msg.data and self.hasObstacle:
            self.freezePathFollowerPublisher.publish(BoolMsg(False))
            self.turnThread.stopTurn()
          
        self.hasObstacle = msg.data
    
    
    def destroy(self):
        self.turnThread.kill = True
    
    
if __name__ == "__main__":
    robotManager = RobotManager()
    
    # ros node
    rospy.init_node("robot_manager", log_level = rospy.INFO)
    rospy.Subscriber("wanted_cmd_vel", TwistMsg, robotManager.wantedCmdVelCB)
    rospy.Subscriber("freeze", BoolMsg, robotManager.freezeCB)
    rospy.Subscriber("front_obstacle", BoolMsg, robotManager.frontObstacleCB)
    K_LINEAR = rospy.get_param("robot_manager_k_linear", 0.5)
    KL_ANGULAR = rospy.get_param("robot_manager_kl_angular", 2.0)
    KA_ANGULAR = rospy.get_param("robot_manager_ka_angular", 1.0)
    rospy.loginfo("Coefficient for K_LINEAR, KL_ANGULAR, KA_ANGULAR : " + 
                  str(K_LINEAR) + "\t" + str(KL_ANGULAR) + "\t" + str(KA_ANGULAR) + "\t")
    rospy.spin()
    
    robotManager.destroy()
