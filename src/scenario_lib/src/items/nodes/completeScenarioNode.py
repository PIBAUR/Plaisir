#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

from PyQt4.QtGui import *

import rospy
import tf

from geometry_msgs.msg import Pose2D as Pose2DMsg

from src.scenario_lib.src.items.robot import Robot
from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.nodes.travelScenarioNode import TravelScenarioNode
from src.scenario_lib.src.items.nodes.choregraphicScenarioNode import ChoregraphicScenarioNode

from src.bezier_curve.src import bezier_interpolate
from path_checker.srv import PathCheckerReq as PathCheckerReq

class CompleteScenarioNode(DiagramNode):
    nodeName = "Sc. complet"
    nodeCategory = ""
    
    maxInputs = 2
    minInputs = 2
    hasOutput = 1
    
    def __init__(self, robotId, parent, canvas, position):
        super(CompleteScenarioNode, self).__init__(robotId, parent, canvas, position)
        
        self.currentInputIndex = 0
        
    
    def output(self, args, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        inputs = self.getInputs()

        robotIndex = self.getRobotIndex()
                
        # get the position of the start of choregraphic scenario to reach this point with the travel scenario
        choregraphicScenario = inputs[1].output(args, None)
        
        if self.currentInputIndex == 0:
            # get scale
            scale = 1. / float(choregraphicScenario.gridSize)
            
            # get the line between the first point of the curve and the target
            if robotIndex > len(choregraphicScenario.robots):
                raise NodeException(self, u"Aucun point n'existe sur le scénario")
                
            robot = choregraphicScenario.robots[robotIndex]
            
            if len(robot.points) <= 0:
                # don't execute this node
                self.stopExecution()
                self.updateCallback(1, True)
                return None
            
            firstPoint = robot.points[0].anchor
            targetPoint = choregraphicScenario.targetPosition
            directionLineVector = ((targetPoint[0] - firstPoint.x()) * scale, (targetPoint[1] - firstPoint.y()) * scale)
            
            # calculate the origin of the choregraphic scenario
            if not "targetPosition" in args.keys():
                raise NodeException(self, u"Le scénario de déplacement doit se siter après un noeud \"Visiteur\" ou \"Scénario complet\"")
            
            # set the target position to the start point of the choregraphic scenario
            originPosition = (args["targetPosition"][0] - directionLineVector[0], args["targetPosition"][1] + directionLineVector[1])
            # get the orientation: the angle between the robot position and the target point
            orientation = math.atan2((args["targetPosition"][1] - args["robotPosition"][1]), (args["targetPosition"][0] - args["robotPosition"][0]))
            
            # keep target position and rotation to give to the path_checker
            choregraphicScenarioTargetPosition = args["targetPosition"]
            
            # transform the point depending on the orientation wanted
            transformOrientation = tf.transformations.quaternion_from_euler(0, 0, orientation)
            scenarioMsg = choregraphicScenario.robots[robotIndex].getScenarioMsgWithParams((originPosition[0], originPosition[1], 0), scale, transformOrientation, True, True)
            args["targetPosition"] = (scenarioMsg.bezier_paths.curves[0].anchor_1.x, scenarioMsg.bezier_paths.curves[0].anchor_1.y, 0)
            #scenarioStartOrientation = bezier_interpolate.getBezierCurveTangentResult(0, scenarioMsg.bezier_paths.curves[0])
            args["targetOrientation"] = orientation#TODO: set the correct orientation
            
            #print "scenarioStartOrientation"
            #print (scenarioStartOrientation / math.pi) * 180.
            
            # store values for absolute coords for choregraphic scenario
            self.targetOrientation = transformOrientation
            self.targetPosition = args["targetPosition"]
            
            path, distance = bezier_interpolate.getPathAndDistanceFromMessage(scenarioMsg, None)
            # check path with service
            rospy.loginfo("try to check path")
            rospy.wait_for_service('path_checker')
            pathChecker = rospy.ServiceProxy('path_checker', PathCheckerReq)
            target = Pose2DMsg(choregraphicScenarioTargetPosition[0], choregraphicScenarioTargetPosition[1], args["targetOrientation"])
            try:
                checkResult = pathChecker(path.path, target)
                rospy.loginfo("check result: " + str(checkResult.is_possible))
                hasCheckResult = checkResult.is_possible
            except rospy.service.ServiceException, e:
                hasCheckResult = False
                rospy.logerr(e.message)
            
            if hasCheckResult:
                # define the first point of path finding with the new path checked
                firstCheckedPathPose = checkResult.path_result.poses[0]
                args["targetPosition"] = (firstCheckedPathPose.position.x, firstCheckedPathPose.position.y, 0)
                args["targetOrientation"] = tf.transformations.euler_from_quaternion([firstCheckedPathPose.orientation.x, firstCheckedPathPose.orientation.y, firstCheckedPathPose.orientation.z, firstCheckedPathPose.orientation.w])[-1]
                print "hasCheckResult"
                print (args["targetOrientation"] / math.pi) * 180.
                
                # store checked path to execute it for choregraphic one
                self.checkedChoregraphicPath = checkResult.path_result#TODO: n'envoyer que s'il y a eu un changement
            else:
                # don't execute this node
                self.stopExecution()
                self.updateCallback(1, True)
                return None
            
        # continue output routine
        if choregraphicScenario.scenarioType == "choregraphic":
            if len(inputs) == 2:
                inputItem = inputs[self.currentInputIndex]
                
                if updateRatioCallback is not None:
                    # not dry run
                    self.startExecution(self.getInputWidgetIndexFromInputIndex(self.currentInputIndex))
                
                scenarioResult = inputItem.output(args, self.updateRatio if updateRatioCallback is not None else None)
                
                # give start pose to execute the choregraphic with absolute position 
                if self.currentInputIndex == 1:
                    scenarioResult.startPosition = self.targetPosition
                    scenarioResult.startOrientation = self.targetOrientation
                    scenarioResult.checkedChoregraphicPath = self.checkedChoregraphicPath
                    
                return scenarioResult
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
