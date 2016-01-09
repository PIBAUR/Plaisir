#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import math

from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException

class RandomChoiceNode(DiagramNode):
    nodeName = u"Aléatoire"
    nodeCategory = ""
    
    maxInputs = 50
    minInputs = 1
    hasOutput = 1
    SIZE_RATIO = 0.50 #2/3
    
    
    def __init__(self, robotId, parent, canvas, position):
        super(RandomChoiceNode, self).__init__(robotId, parent, canvas, position)
        
        self.randomIndex = -1
        self.lastInputsPlayed = []
    
    def output(self, args, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()
        
        #if self.randomIndex == -1:
        """
        Random selector :
            chose randomly the next scenario and check if it is one of the last X scenario played
            X may be absolute or define according to the number of inputs
        """
        # update last played list
        # "while" in case of list of inputs is shorter than previous iteration
        self.lastInputsPlayed.append(self.randomIndex)
        while len(self.lastInputsPlayed) > math.floor(len(inputs)*self.SIZE_RATIO):
            self.lastInputsPlayed.pop(0)
        # init new index
        self.randomIndex = self.lastInputsPlayed[0]
        # loop while new index is one of the last scenarios played
        while self.randomIndex in self.lastInputsPlayed:
            self.randomIndex = random.randint(0,len(inputs)-1)
        """
        End of modification
        """
        
        inputItem = inputs[self.randomIndex]
        
        if updateRatioCallback is not None:
            # dry run
            self.startExecution(self.getInputWidgetIndexFromInputIndex(self.randomIndex))
        
        return inputItem.output(args, self.updateRatio if updateRatioCallback is not None else None)
    
    
    def updateRatio(self, inputRatio, paused):
        if inputRatio >= 1 or paused:
            # if input is complete, change random
            #if inputRatio >= 1:
                #self.randomIndex = -1
            
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
        
        