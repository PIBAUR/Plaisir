#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from PyQt4.QtGui import *

from std_msgs.msg import String as StringMsg

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException

class BatteryStateNode(DiagramNode):
    nodeName = "Etat de la batterie"
    nodeCategory = ""
    
    maxInputs = 2
    minInputs = 2
    hasOutput = 1
    
    def __init__(self, parent, canvas, position):
        super(BatteryStateNode, self).__init__(parent, canvas, position)
        
        # topic
        self.stateSubscriber = rospy.Subscriber("robot01/state", StringMsg, self.handleStateReceived)
        
        # ui
        self.isLowBatteryState = False
        self.isLowBatteryState_label = QLabel()
        self.refreshIsLowBatteryStateLabel()
        self.widget.central_widget.layout().addWidget(self.isLowBatteryState_label)
        
    
    def output(self, args, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()
        if len(inputs) != 2:
            raise NodeException(u"Le noeud d'état de la batterie doit comporter 2 entrées; 1: si le niveau de batterie est suffisant; 2: si non")
        
        inputIndex = 1 if self.isLowBatteryState else 0
        
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
    
    
    def handleStateReceived(self, msg):
        self.isLowBatteryState = msg.data == "LOW_BATTERY"
            
        self.refreshIsLowBatteryStateLabel()
    
    
    def refreshIsLowBatteryStateLabel(self):
        self.isLowBatteryState_label.setText("insuffisante" if self.isLowBatteryState else "suffisante")
        
        
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
        
        