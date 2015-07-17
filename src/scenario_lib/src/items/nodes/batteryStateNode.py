#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.nodes.playNode import PlayNode

class BatteryStateNode(DiagramNode):
    nodeName = "Etat de la batterie"
    nodeCategory = ""
    
    maxInputs = 2
    minInputs = 2
    hasOutput = 1
    
    def __init__(self, robotId, parent, canvas, position):
        super(BatteryStateNode, self).__init__(parent, robotId, canvas, position)
        
        #TODO: pourcentage ascendant et descendant
        #TODO: subscriber
        self.batteryPercent = 100
        
        # ui
        self.isLowBatteryState_label = QLabel()
        self.widget.central_widget.layout().addWidget(self.isLowBatteryState_label)
        
    
    def output(self, args, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()
        if len(inputs) != 2:
            raise NodeException(u"Le noeud d'état de la batterie doit comporter 2 entrées; 1: si le niveau de batterie est suffisant; 2: si non")
        
        inputIndex = 1 if self.batteryPercent > 20 else 0
        
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
        self.isLowBatteryState_label.setText(str(self.batteryPercent))
        
        super(BatteryStateNode, self).refreshUI(args)
        
        
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
        
        