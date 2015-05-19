#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4.QtGui import *

import rospy
import tf
from functools import partial

from std_msgs.msg import String as StringMsg
from std_msgs.msg import Header as HeaderMsg
from std_msgs.msg import Bool as BoolMsg
from scenario_msgs.msg import Scenario as ScenarioMsg
from scenario_msgs.msg import BezierPath as BezierPathMsg
from scenario_msgs.msg import BezierCurve as BezierCurveMsg

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.choregraphicScenarioNode import ChoregraphicScenarioNode
from src.scenario_lib.src.items.nodes.scenarioNode import ScenarioNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from PyQt4.Qt import QTimer

class PlayNode(DiagramNode):
    INTERRUPTED_STATE = "INTERRUPTED_STATE"
    
    transformListener = None
    actionClient = None
    
    nodeName = "Play"
    nodeCategory = ""
    
    maxInputs = 1
    minInputs = 1
    hasOutput = 0
    
    def __init__(self, parent, canvas, position):
        super(PlayNode, self).__init__(parent, canvas, position)
        
        self.isPlaying = False
        self.playingScenario = None
        self.transformPosition = (0, 0, 0)
        self.transformOrientation = (0, 0, 0, 1)
        
        # set ROS scenarioPublisher to publish scenarios
        self.scenarioPublisher = rospy.Publisher('/robot01/scenario', ScenarioMsg)
        self.freezePublisher = rospy.Publisher('/robot01/freeze', BoolMsg)
        
        # set ROS subscriber to get informations from the robot
        if PlayNode.transformListener is None:
            PlayNode.transformListener = tf.TransformListener()
        
        # state
        self.stateSubscriber = rospy.Subscriber("robot01/state", StringMsg, self.handleStateReceived)
        
        # timer to not execute play or stop in an other thread
        self.hasToRestart = False
        self.threadSafeTimer = QTimer()
        self.threadSafeTimer.timeout.connect(partial(self.handleThreadSafeTimer))
        self.threadSafeTimer.start(20)
        
        # ui
        self.playButton = QPushButton("Play")
        self.playButton.clicked.connect(self.handlePlayButtonClicked)
        self.widget.central_widget.layout().addWidget(self.playButton)
        self.simulateButton = QPushButton("Simulate")
        self.simulateButton.clicked.connect(self.handleSimulateButtonClicked)
        self.widget.central_widget.layout().addWidget(self.simulateButton)
        self.playingScenarioLabel = QLabel("")
        self.widget.central_widget.layout().addWidget(self.playingScenarioLabel)
        self.stopButton = QPushButton("Stop")
        self.stopButton.clicked.connect(self.handleStopButtonClicked)
        self.stopButton.setEnabled(False)
        self.widget.central_widget.layout().addWidget(self.stopButton)
    
    
    def output(self):
        self.stopExecution()
        
        inputs = self.getInputs()
        
        self.startExecution(0)
        
        return inputs[0].output(self.getArgs(), self.updateRatio)
    
    
    def getArgs(self):
        return {
                "robotPosition": self.transformPosition, 
                "robotOrientation": self.transformOrientation
                }
    
    
    def updateRatio(self, inputRatio, paused):
        if paused:
            self.playScenario()
        else:
            # update ui
            self.setTimelineValue(inputRatio)
            
            # get transform
            self.getRobotTransform()
    
    
    def getRobotTransform(self):
        try:
            self.transformPosition, self.transformOrientation = self.transformListener.lookupTransform('/map', '/robot01/base_link', rospy.Time(0))
        except Exception, e:
            rospy.logerr(e)
        
    
    def stop(self, withoutSendingFreeze = False):
        if not withoutSendingFreeze and self.executing:
            # stop the robot
            freezeMsg = BoolMsg()
            freezeMsg.data = True
            self.freezePublisher.publish(freezeMsg)
        
        super(PlayNode, self).stop()
        
        self.playingScenario = None
        self.playingScenarioLabel.setText("")
        
    
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
    
    
    def playScenario(self):
        #DEBUG: remove exception handler
        try:
            # reset error
            self.canvas.ui.statusBar.clearMessage()
            for nodeInstance in self.canvas.nodesInstances:
                nodeInstance.widget.central_widget.setStyleSheet("#central_widget { background: #fff; }")
                
            # play
            self.playingScenario = self.output()
            self.playingScenarioLabel.setText(self.playingScenario.niceName())
            
            # publish message to ROS
            if self.playingScenario.scenarioType == "choregraphic":
                scale = 1. / float(self.playingScenario.gridSize)
                interpolation = True
            else:
                scale = 1
                interpolation = False
            
            robot = self.playingScenario.robots[0]
            if self.playingScenario.startPosition is not None:
                transformPosition = self.playingScenario.startPosition
            else:
                transformPosition = self.transformPosition
                
            if self.playingScenario.startOrientation is not None:
                transformOrientation = self.playingScenario.startOrientation
            else:
                transformOrientation = self.transformOrientation
            
            scenarioMsg = robot.getScenarioMsgWithParams(transformPosition, scale, transformOrientation, interpolation, False)
            scenarioMsg.uid = self.playingScenario.uid
            scenarioMsg.type = self.playingScenario.scenarioType
            self.scenarioPublisher.publish(scenarioMsg)
        except NodeException as error:
            self.playButton.setEnabled(True)
            self.stopButton.setEnabled(False)
            
            # display the error
            self.canvas.ui.statusBar.showMessage("Erreur: " + str(error.message.encode("utf-8")).decode("utf-8"))
            error.nodeCausingErrror.widget.central_widget.setStyleSheet("#central_widget { background: #ff4c4c; }")
            
            self.stopAllScenarios()
    
    
    def handleThreadSafeTimer(self):
        if self.hasToRestart:
            self.hasToRestart = False
            if self.isPlaying:
                self.stopAllScenarios(True)
                self.playScenario()
                
        # refresh ui
        self.refreshUI(self.getArgs())
        
        
    def handleStateReceived(self, msg):
        state = msg.data
        
        if state == PlayNode.INTERRUPTED_STATE:
            self.hasToRestart = True
    
    
    def stopAllScenarios(self, withoutSendingFreeze = False):
        for nodeInstance in self.canvas.nodesInstances:
            if self == nodeInstance:
                nodeInstance.stop(withoutSendingFreeze)
            else:  
                nodeInstance.stop()
        
    
    def handlePlayButtonClicked(self, event):
        self.isPlaying = True
        
        self.playButton.setEnabled(False)
        self.stopButton.setEnabled(True)
        
        ScenarioNode.simulation = False
        self.playScenario()
        
        
    def handleSimulateButtonClicked(self, event):
        self.isPlaying = True
        
        self.playButton.setEnabled(False)
        self.stopButton.setEnabled(True)
        
        ScenarioNode.simulation = True
        self.playScenario()
        
        
    def handleStopButtonClicked(self, event):
        self.isPlaying = False
        
        self.playButton.setEnabled(True)
        self.stopButton.setEnabled(False)
        
        self.stopAllScenarios()
