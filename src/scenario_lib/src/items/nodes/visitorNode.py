#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from PyQt4.QtGui import *

from scenario_msgs.msg import ObstacleArray as ObstacleArrayMsg

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException

class VisitorNode(DiagramNode):
    nodeName = "Visiteur"
    nodeCategory = ""
    
    maxInputs = 2
    minInputs = 2
    hasOutput = 1
    
    def __init__(self, parent, canvas, position):
        super(VisitorNode, self).__init__(parent, canvas, position)
        
        self.args = {}
        
        # topic
        self.visitorSubscriber = rospy.Subscriber("/obstacles", ObstacleArrayMsg, self.handleObstaclesReceived)
        
        # ui
        self.visitor = None
        self.isVisitor_label = QLabel()
        self.refreshIsVisitorLabel()
        self.widget.central_widget.layout().addWidget(self.isVisitor_label)
        
    
    def output(self, args, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()
        if len(inputs) != 2:
            raise NodeException(u"Le noeud de visiteur doit comporter 2 entrées; 1: s'il y a un visiteur; 2: si non")
        
        inputIndex = 0 if self.visitor is not None else 1
        
        if self.visitor is not None:
            inputIndex = 0
            args["targetPosition"] = (self.visitor.x, self.visitor.y)
        else:
            inputIndex = 1
        
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
    
    
    def handleObstaclesReceived(self, msg):
        if len(msg.obstacles) > 0:
            self.visitor = msg.obstacles[0]
        else:
            self.visitor = None
            
        self.refreshIsVisitorLabel()
    
    
    def refreshIsVisitorLabel(self):
        self.isVisitor_label.setText("visiteur: " + ("oui" if self.visitor is not None else "non"))
        
        
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
        
        