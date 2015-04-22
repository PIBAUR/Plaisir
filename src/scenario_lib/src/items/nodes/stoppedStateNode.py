#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.nodes.playNode import PlayNode

class StoppedStateNode(DiagramNode):
    nodeName = u"Obstacle devant"
    nodeCategory = ""
    
    maxInputs = 2
    minInputs = 2
    hasOutput = 1
    
    def __init__(self, parent, canvas, position):
        super(StoppedStateNode, self).__init__(parent, canvas, position)
        
        # ui
        self.isStoppedState_label = QLabel()
        self.widget.central_widget.layout().addWidget(self.isStoppedState_label)
        
    
    def output(self, args, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()
        if len(inputs) != 2:
            raise NodeException(u"Le noeud de redémarrage doit comporter 2 entrées; 1: si le robot a du être interomppu; 2: si non")
        
        isStoppedState = args["currentState"] == PlayNode.STOP_STATE
        inputIndex = 1 if isStoppedState else 0
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
        isStoppedState = args["currentState"] == PlayNode.STOP_STATE
        self.isStoppedState_label.setText("oui" if isStoppedState else "non")
        
        super(StoppedStateNode, self).refreshUI(args)
        
        
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
        
        