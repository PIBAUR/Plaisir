#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

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
        
        #DEBUG: only for viz
        import rospy
        from scenario_msgs.msg import Scenario as ScenarioMsg
        self.scenarioPublisher = rospy.Publisher('/robot01/scenario', ScenarioMsg)
        
    
    def output(self, args, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()
        
        # get the position of the start of choregraphic scenario to reach this point with the travel scenario
        choregraphicScenario = inputs[1].output(args, None)
        
        if self.currentInputIndex == 0:
            # get scale
            scale = 1. / float(choregraphicScenario.gridSize)
            
            # get the line between the first point of the curve and the target
            robot = choregraphicScenario.robots[0]
            firstPoint = robot.points[0].anchor
            targetPoint = choregraphicScenario.targetPosition
            directionLineVector = ((targetPoint[0] - firstPoint.x()) * scale, (targetPoint[1] - firstPoint.y()) * scale)
            
            # calculate the origin of the choregraphic scenario
            if not "targetPosition" in args.keys():
                raise NodeException(self, u"Le scénario de déplacement doit se siter après un noeud \"Visiteur\" ou \"Scénario complet\"")
            
            originPosition = (args["targetPosition"][0] - directionLineVector[0], args["targetPosition"][1] - directionLineVector[1])
            orientation = (1, 0, 0, 0)
            
            """
            print robot.getScenarioMsg(originPosition, scale, orientation).bezier_paths.curves.anchor_1.x
            print robot.getScenarioMsg(originPosition, scale, orientation).bezier_paths.curves.anchor_1.y
            print "=="
            transformationMatrix = robot.getTransformationMatrix(scale, orientation)
            x, y, z, w = self.getTransformedPoint(bezierCurve.anchor_1.x, bezierCurve.anchor_1.y, transformationMatrix, transformPosition)
            print x
            print y
            print "_____________________"
            """
            #DEBUG: only for viz
            #self.scenarioPublisher.publish(choregraphicScenario.robots[0].getScenarioMsg(originPosition, scale, orientation))
        
        # continue output routine
        if choregraphicScenario.scenarioType == "choregraphic":
            if len(inputs) == 2:
                inputItem = inputs[self.currentInputIndex]
                
                if updateRatioCallback is not None:
                    # not dry run
                    self.startExecution(self.getInputWidgetIndexFromInputIndex(self.currentInputIndex))
                
                return inputItem.output(args, self.updateRatio if updateRatioCallback is not None else None)
            else:
                raise(NodeException(self, u"Les 2 entrées doivent être liées à un scénario de déplacement puis un scénario chorégraphique"))
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
