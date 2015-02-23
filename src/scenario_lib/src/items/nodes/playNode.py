#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4.QtGui import *

import rospy
import tf

from scenario_msgs.msg import Scenario as ScenarioMsg
from scenario_msgs.msg import BezierPath as BezierPathMsg
from scenario_msgs.msg import BezierCurve as BezierCurveMsg
from std_msgs.msg import Header as HeaderMsg

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException

class PlayNode(DiagramNode):
    transformListener = None
    actionClient = None
    
    nodeName = "Play"
    nodeCategory = ""
    
    maxInputs = 1
    minInputs = 1
    hasOutput = 0
    inputGroup = [""]
    
    def __init__(self, parent, canvas, position):
        super(PlayNode, self).__init__(parent, canvas, position)
        
        self.playingScenario = None
        self.transformPosition = (0, 0, 0)
        self.transformOrientation = (1, 0, 0, 0)
        
        # set ROS scenarioPublisher to publish scenarios
        self.scenarioPublisher = rospy.Publisher('scenario', ScenarioMsg)
        
        # set ROS subscriber to get informations from the robot
        if PlayNode.transformListener is None:
            PlayNode.transformListener = tf.TransformListener()
        
        self.playButton = QPushButton("Play")
        self.playButton.clicked.connect(self.handlePlayButtonClicked)
        self.widget.central_widget.layout().addWidget(self.playButton)
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
        
        return inputs[0].output(self.updateRatio)
    
    
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
        except:
            pass
        
    
    def stop(self):
        # stop the robot
        if self.executing:
            emptyScenarioMsg = ScenarioMsg()
            headerMsg = HeaderMsg()
            emptyScenarioMsg.bezier_paths = BezierPathMsg()
            emptyScenarioMsg.bezier_paths.curves = []
            
            #TODO: convert video_player to media_player
            #self.scenarioMsg.video_player = VideoPlayerMsg()
            #self.scenarioMsg.video_player.video_paths = ["test_video.mp4"]
            headerMsg.frame_id = "/map"
            headerMsg.stamp = rospy.Time.now()
            emptyScenarioMsg.bezier_paths.header = headerMsg
            self.scenarioPublisher.publish(emptyScenarioMsg)
        
        
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
            scenarioMsg = self.playingScenario.robots[0].getScenarioMsg(self.transformPosition, scale, self.transformOrientation)
            scenarioMsg.uid = self.playingScenario.uid
            self.scenarioPublisher.publish(scenarioMsg)
        except Exception, error:
            self.playButton.setEnabled(True)
            self.stopButton.setEnabled(False)
            
            # display the error
            self.canvas.ui.statusBar.showMessage("Erreur: " + error.message)
            if error.__class__ == NodeException:
                error.nodeCausingErrror.widget.central_widget.setStyleSheet("#central_widget { background: #ff4c4c; }")
            
            self.stopAllScenarios()
    
    
    def stopAllScenarios(self):
        for nodeInstance in self.canvas.nodesInstances:
            nodeInstance.stop() 
        
    
    def handlePlayButtonClicked(self, event):
        self.playButton.setEnabled(False)
        self.stopButton.setEnabled(True)
        
        self.playScenario()
        
        
    def handleStopButtonClicked(self, event):
        self.playButton.setEnabled(True)
        self.stopButton.setEnabled(False)
        
        self.stopAllScenarios()