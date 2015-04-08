#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4.QtGui import *

import rospy
import tf

from scenario_msgs.msg import Scenario as ScenarioMsg
from scenario_msgs.msg import BezierPath as BezierPathMsg
from scenario_msgs.msg import BezierCurve as BezierCurveMsg
from std_msgs.msg import Header as HeaderMsg
from std_msgs.msg import Bool as BoolMsg

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.choregraphicScenarioNode import ChoregraphicScenarioNode
from src.scenario_lib.src.items.nodes.scenarioNode import ScenarioNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException

class PlayNode(DiagramNode):
    transformListener = None
    actionClient = None
    
    nodeName = "Play"
    nodeCategory = ""
    
    maxInputs = 1
    minInputs = 1
    hasOutput = 0
    
    def __init__(self, parent, canvas, position):
        super(PlayNode, self).__init__(parent, canvas, position)
        
        self.playingScenario = None
        self.transformPosition = (0, 0, 0)
        self.transformOrientation = (1, 0, 0, 0)
        
        # set ROS scenarioPublisher to publish scenarios
        self.scenarioPublisher = rospy.Publisher('/robot01/scenario', ScenarioMsg)
        self.freezePublisher = rospy.Publisher('/robot01/freeze', BoolMsg)
        
        # set ROS subscriber to get informations from the robot
        if PlayNode.transformListener is None:
            PlayNode.transformListener = tf.TransformListener()
        
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
        
        return inputs[0].output({}, self.updateRatio)
    
    
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
        
    
    def stop(self):
        if self.executing:
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
        try:
            # reset error
            self.canvas.ui.statusBar.clearMessage()
            for nodeInstance in self.canvas.nodesInstances:
                nodeInstance.widget.central_widget.setStyleSheet("#central_widget { background: #fff; }")
                
            # play
            self.playingScenario = self.output()
            self.playingScenarioLabel.setText(self.playingScenario.niceName())
            
            # publish message to ROS
            scale = 1. / float(self.playingScenario.gridSize)
            scenarioMsg = self.playingScenario.robots[0].getScenarioMsg(self.transformPosition, scale, self.transformOrientation, self.playingScenario.scenarioType == "choregraphic")
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
    
    
    def stopAllScenarios(self):
        for nodeInstance in self.canvas.nodesInstances:
            nodeInstance.stop() 
        
    
    def handlePlayButtonClicked(self, event):
        self.playButton.setEnabled(False)
        self.stopButton.setEnabled(True)
        
        ScenarioNode.simulation = False
        self.playScenario()
        
        
    def handleSimulateButtonClicked(self, event):
        self.playButton.setEnabled(False)
        self.stopButton.setEnabled(True)
        
        ScenarioNode.simulation = True
        self.playScenario()
        
        
    def handleStopButtonClicked(self, event):
        self.playButton.setEnabled(True)
        self.stopButton.setEnabled(False)
        
        self.stopAllScenarios()