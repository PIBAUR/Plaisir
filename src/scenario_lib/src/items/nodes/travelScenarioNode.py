#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
import tf

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from geometry_msgs.msg import Pose2D
from src.scenario_lib.src.items.nodes.scenarioNode import ScenarioNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.scenario import Scenario
from src.scenario_lib.src.items.curvePoint import CurvePoint
from src.scenario_lib.src.items.point import Point
from src.gui_scenario_db.src.ui import ScenarioDataBase

from path_finding.srv import PathFinding as PathFindingSrv

class TravelScenarioNode(ScenarioNode):
    nodeName = u"Sc. dépl."
    
    def __init__(self, parent, canvas, position):
        super(TravelScenarioNode, self).__init__(parent, canvas, position)
    
    
    def output(self, args, updateRatioCallback):
        if not "targetPosition" in args.keys():
            raise NodeException(self, u"Le scénario de déplacement doit se siter après un noeud \"Visiteur\" ou \"Scénario complet\"")
        
        targetPosition = args["targetPosition"]
        
        # get path finding
        rospy.wait_for_service('path_finding')
        pathFinding = rospy.ServiceProxy('path_finding', PathFindingSrv)
        target = Pose2D(targetPosition[0], targetPosition[1], 0)
        pathResult = pathFinding(target)
        
        # set scenario
        self.currentScenario = Scenario(False)
        self.currentScenario.scenarioType = "travel"
        self.currentScenario.gridSize = 1
        
        for pose in pathResult.path.poses:
            curvePoint = CurvePoint(Point(pose.x, pose.y, pose.theta))
            self.currentScenario.robots[0].points.append(curvePoint)
        
        self.currentScenario.name = "-> " + str(math.floor(targetPosition[0] * 10) / 10) + ";" + str(math.floor(targetPosition[1] * 10) / 10)
        
        return super(TravelScenarioNode, self).output(args, updateRatioCallback)
        
        
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
    