#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random

from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException

class MasteringNode(DiagramNode):
    nodeName = u"Mastering"
    nodeCategory = ""
    
    maxInputs = 50
    minInputs = 1
    hasOutput = 1
    
    def __init__(self, robotId, parent, canvas, position):
        super(MasteringNode, self).__init__(robotId, parent, canvas, position)
        
    
    def output(self, args, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()
        
        indexOutput = 0
        i = 0
        if args["isMastering"] is not None and args["isMastering"] != False:
            for inputNode in inputs:
                if args["isMastering"] == inputNode.masterId and inputNode.robotId == self.robotId:
                    indexOutput = i
                    break
                
                i += 1
        
        inputItem = inputs[indexOutput]
        
        if updateRatioCallback is not None:
            # dry run
            self.startExecution(self.getInputWidgetIndexFromInputIndex(indexOutput))
        
        return inputItem.output(args, self.updateRatio if updateRatioCallback is not None else None)
    
    
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
        
        