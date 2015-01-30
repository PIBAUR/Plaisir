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
        
    
    def output(self, updateRatioCallback, updateOutputCallback, firstCall = True):
        self.updateCallback = updateRatioCallback
        self.updateOutputCallback = updateOutputCallback
        
        if firstCall:
            self.currentInputIndex = 0
        else:
            self.currentInputIndex += 1
        
        inputs = super(SequenceNode, self).output()
        self.numInputs = len(inputs)
        
        inputItem = inputs[self.currentInputIndex]
        
        return inputItem.output(self.updateRatio, self.updateOutputCallback)
    
    
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
    
    
    def updateRatio(self, ratio):
        # while it is not the last index; continue
        if self.currentInputIndex < self.numInputs - 1 and ratio == 1:
            self.updateOutputCallback(self.output(self.updateCallback, self.updateOutputCallback, False))
            return
        
        ratio = float(ratio)
        sequenceRatio = (float(self.currentInputIndex) / (self.numInputs)) + (ratio / self.numInputs)
        
        self.setTimelineValue(sequenceRatio)
        
        self.updateCallback(sequenceRatio)