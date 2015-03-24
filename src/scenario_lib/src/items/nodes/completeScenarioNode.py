#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.nodes.travelScenarioNode import TravelScenarioNode
from src.scenario_lib.src.items.nodes.choregraphicScenarioNode import ChoregraphicScenarioNode

class CompleteScenarioNode(DiagramNode):
    nodeName = "Sc. complet"
    nodeCategory = ""
    
    maxInputs = 2
    minInputs = 2
    hasOutput = 1
    
    def __init__(self, parent, canvas, position):
        super(CompleteScenarioNode, self).__init__(parent, canvas, position)
        
        self.currentInputIndex = 0
        
    
    def output(self, args, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()
        
        # get the position of the start of choregraphic scenario to reach this point with the travel scenario
        choregraphicScenario = inputs[1].output(args, None)
        
        #TODO: calculer le point cible par rapport au scénario chorégraphique qui vient
        args["targetPosition"] = choregraphicScenario.targetPosition
        
        if choregraphicScenario.name is not None:
            if len(inputs) == 2:
                inputItem = inputs[self.currentInputIndex]
                
                if updateRatioCallback is not None:
                    # dry run
                    self.startExecution(self.getInputWidgetIndexFromInputIndex(self.currentInputIndex))
                
                return inputItem.output(args, self.updateRatio if updateRatioCallback is not None else None)
        else:
            raise(NodeException(self, u"L'entrée 2 doit être liée à un scénario chorégraphique"))
    
    
    def updateRatio(self, inputRatio, paused):
        inputRatio = float(inputRatio)
        
        # get sequence ratio
        sequenceRatio = (float(self.currentInputIndex) / 2) + (inputRatio / 2)
        
        if inputRatio >= 1:
            # reset index
            self.currentInputIndex += 1
            if self.currentInputIndex >= 2:
                self.currentInputIndex = 0
        
        if sequenceRatio >= 1:
            # stop sequence
            self.stopExecution()
            self.updateCallback(1, True)
        else:
            if inputRatio >= 1 or paused:
                # send pause
                self.stopExecution()
                self.updateCallback(sequenceRatio, True)
            else:
                # continue
                self.updateCallback(sequenceRatio, False)
                self.setTimelineValue(sequenceRatio)
    
    
    def stop(self):
        super(CompleteScenarioNode, self).stop()
        
        self.currentInputIndex = 0
    
    
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
