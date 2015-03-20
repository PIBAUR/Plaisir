#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random

from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException

class RandomChoiceNode(DiagramNode):
    nodeName = u"Aléatoire"
    nodeCategory = ""
    
    maxInputs = 50
    minInputs = 1
    hasOutput = 1
    
    def __init__(self, parent, canvas, position):
        super(RandomChoiceNode, self).__init__(parent, canvas, position)
        
        self.randomIndex = 0
        
    
    def output(self, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()
        
        self.randomIndex = random.randint(0, len(inputs) - 1)
        inputItem = inputs[self.randomIndex]
        
        self.startExecution(self.getInputWidgetIndexFromInputIndex(self.randomIndex))
        
        return inputItem.output(self.updateRatio)
    
    
    def updateRatio(self, inputRatio, paused):
        if inputRatio >= 1 or paused:
            self.stopExecution()
            self.updateCallback(inputRatio, True)
        else:
            self.updateCallback(inputRatio, False)
            self.setTimelineValue(inputRatio)
        return inputRatio
    
    
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
        
        