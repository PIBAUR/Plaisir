#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException

class SequenceNode(DiagramNode):
    nodeName = "Sequence"
    nodeCategory = ""
    
    maxInputs = 4
    minInputs = 1
    hasOutput = 1
    inputGroup = [""]
    
    def __init__(self, parent, canvas, position):
        super(SequenceNode, self).__init__(parent, canvas, position)
        
        self.currentInputIndex = 0
        self.numInputs = 0
        
    
    def output(self, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()
        self.numInputs = len(inputs)
        
        if self.currentInputIndex > len(inputs) - 1: # in case of intended index has been removed
            self.currentInputIndex = 0
        inputItem = inputs[self.currentInputIndex]
        
        self.startExecution(self.getInputWidgetIndexFromInputIndex(self.currentInputIndex))
        
        return inputItem.output(self.updateRatio)
    
    
    def updateRatio(self, inputRatio, paused):
        inputRatio = float(inputRatio)
        
        sequenceRatio = (float(self.currentInputIndex) / (self.numInputs)) + (inputRatio / self.numInputs)
        
        if inputRatio >= 1:
            self.currentInputIndex += 1
            if self.currentInputIndex >= self.numInputs:
                self.currentInputIndex = 0
        
        if sequenceRatio >= 1:
            self.stopExecution()
            self.updateCallback(1, True)
        else:
            if inputRatio >= 1 or paused:
                self.stopExecution()
                self.updateCallback(sequenceRatio, True)
            else:
                self.updateCallback(sequenceRatio, False)
                self.setTimelineValue(sequenceRatio)
    
    
    def stop(self):
        super(SequenceNode, self).stop()
        
        self.currentInputIndex = 0
    
    
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
        
        