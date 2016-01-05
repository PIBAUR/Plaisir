#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import math
import time

from PyQt4.QtCore import *
from PyQt4.QtGui import *

import rospy

from scenario_msgs.msg import PathFeedback as PathFeedbackMsg

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.scenario import Scenario
from src.gui_scenario_db.src.ui import ScenarioDataBase


class ScenarioNode(DiagramNode):
    nodeName = ""
    nodeCategory = ""
    
    maxInputs = 0
    minInputs = 0
    hasOutput = 1
    
    uid = 0
    
    simulation = False
    lastPathFeedbackUpdateByRobot = {}
    
    masterPlaying = False #DIRTY DIRTY BEARK BEARK
    
    def __init__(self, robotId, parent, canvas, position):
        super(ScenarioNode, self).__init__(robotId, parent, canvas, position)
        
        if not self.robotId in ScenarioNode.lastPathFeedbackUpdateByRobot:
            ScenarioNode.lastPathFeedbackUpdateByRobot[self.robotId] = time.time()
                
        self.scenarioRunningOnRobotUid = -1
        self.currentScenario = None
        self.pathFeedbackValue = 0.
        self.pathFeedbackSubscriber = None
        
        self.lastFeedbackTime = time.time()
        
        # set uiTimer for update graphics, because they must not be executed by the ROS callback thread
        self.uiTimer = QTimer(self.widget)
        self.uiTimer.timeout.connect(self.handleUITimer)
        self.uiTimer.setInterval(100)
        self.uiTimer.start()
        
        # set uiTimer to simulate
        self.simulationTimer = QTimer(self.widget)
        self.simulationTimer.timeout.connect(self.handleSimulationTimer)
        self.simulationTimer.setInterval(10)
    
    
    def output(self, args, updateRatioCallback):
        if self.currentScenario is None:
            raise NodeException(self, u"aucun scénario n'a été chargé")
        
        if updateRatioCallback is None:
            # dry run
            return self.currentScenario
        else:
            self.updateCallback = updateRatioCallback
            
            self.currentScenario.uid = ScenarioNode.uid
            self.scenarioRunningOnRobotUid = self.currentScenario.uid
            ScenarioNode.uid += 1
            self.startExecution(0)
            
            # init subscription on the path feedback
            if not ScenarioNode.simulation:
                self.pathFeedbackSubscriber = rospy.Subscriber("/" + self.robotId + "/path_feedback", PathFeedbackMsg, self.handlePathFeedbackReceived)
                self.handlePathFeedbackReceived(None)
            else:
                self.pathFeedbackValue = 0
                self.simulationTimer.start()
                self.handleSimulationTimer()
            
            # init args
            self.currentScenario.relative = True
            self.currentScenario.startPosition = None
            self.currentScenario.startOrientation = None
            
            self.currentScenario.originNode = self
            
            return self.currentScenario
    
    
    def stop(self):
        super(ScenarioNode, self).stop()
        
        if not ScenarioNode.simulation:
            if self.pathFeedbackSubscriber is not None:
                try:
                    self.pathFeedbackSubscriber.unregister()
                except:
                    pass
        else:
            self.simulationTimer.stop()
        
    
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
    
    
    def handleUITimer(self, firstTime = False):
        """
        if ScenarioNode.masterPlaying and time.time() - ScenarioNode.lastPathFeedbackUpdateByRobot[self.robotId] > 10:
            # time out ! to relaunch
            print "pathFeedback timeout for " + self.robotId
            for nodeInstance in self.canvas.nodesInstances:
                if nodeInstance.__class__.nodeName == "Play" and nodeInstance.isPlaying and nodeInstance.robotId == self.robotId: #BEARK BEARK BEARK !
                    print "replay PlayNode"
                    nodeInstance.handleStopButtonClicked(None)
                    time.sleep(1)
                    nodeInstance.handlePlayButtonClicked(None)
            ScenarioNode.lastPathFeedbackUpdateByRobot[self.robotId] = time.time()"""
        
        if self.executing:
            if self.pathFeedbackValue >= 1:
                self.stopExecution()
                self.updateCallback(1, True)
                self.pathFeedbackValue = 0
            else:
                if self.updateCallback is not None:
                    self.updateCallback(self.pathFeedbackValue, False)
                self.setTimelineValue(self.pathFeedbackValue)
        
        
    def handleSimulationTimer(self):
        self.pathFeedbackValue += .005
        if self.pathFeedbackValue >= 1:
            self.simulationTimer.stop()
    
    
    def handlePathFeedbackReceived(self, msg):
        ScenarioNode.lastPathFeedbackUpdateByRobot[self.robotId] = time.time()
        if msg is None or math.isnan(msg.ratio):
            self.pathFeedbackValue = 0
        else:
            # check that the feedback comes from the right node
            if self.scenarioRunningOnRobotUid == msg.uid:
                self.pathFeedbackValue = msg.ratio
                
                if self.pathFeedbackValue >= 1:
                    self.pathFeedbackSubscriber.unregister()
