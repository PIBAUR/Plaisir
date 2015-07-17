#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool as BoolMsg

from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.nodes.playNode import PlayNode

class ObstacleNode(DiagramNode):
    nodeName = u"Obstacle devant"
    nodeCategory = ""
    
    maxInputs = 2
    minInputs = 2
    hasOutput = 1
    
    def __init__(self, robotId, parent, canvas, position):
        super(ObstacleNode, self).__init__(robotId, parent, canvas, position)
        
        self.stateSubscriber = rospy.Subscriber("/" + self.robotId + "/front_obstacle", BoolMsg, self.handleFrontObstacleReceived)
        self.isObstacle = False
        
        # ui
        self.isStoppedState_label = QLabel()
        self.widget.central_widget.layout().addWidget(self.isStoppedState_label)
        
    
    def handleFrontObstacleReceived(self, msg):
        self.isObstacle = msg.data
        
        
    def output(self, args, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()
        if len(inputs) != 2:
            raise NodeException(u"Le noeud de redémarrage doit comporter 2 entrées; 1: si le robot a du être interomppu; 2: si non")
        
        inputIndex = 1 if self.isObstacle else 0
        inputItem = inputs[inputIndex]
        
        if updateRatioCallback is not None:
            # dry run
            self.startExecution(self.getInputWidgetIndexFromInputIndex(inputIndex))
        
        return inputItem.output(args, self.updateRatio if updateRatioCallback is not None else None)
    
    
    def updateRatio(self, inputRatio, paused):
        if inputRatio >= 1 or paused:
            self.stopExecution()
            self.updateCallback(inputRatio, True)
        else:
            self.updateCallback(inputRatio, False)
            self.setTimelineValue(inputRatio)
        return inputRatio
    
    
    def refreshUI(self, args):
        self.isStoppedState_label.setText("oui" if self.isObstacle else "non")
        
        super(ObstacleNode, self).refreshUI(args)
        
        
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
        
        