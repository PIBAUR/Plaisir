#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.scenarioNode import ScenarioNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.scenario import Scenario
from src.gui_scenario_db.src.ui import ScenarioDataBase


class TravelScenarioNode(ScenarioNode):
    nodeName = u"Sc. dépl."
    
    def __init__(self, parent, canvas, position):
        super(TravelScenarioNode, self).__init__(parent, canvas, position)
    
    
    def output(self, args, updateRatioCallback):
        self.currentScenario = Scenario()
        if not "targetPosition" in args.keys():
            raise NodeException(self, u"Le scénario de déplacement doit se siter après un noeud \"Visiteur\" ou \"Scénario complet\"")
        
        targetPosition = args["targetPosition"]
        self.currentScenario.name = "-> " + str(math.floor(targetPosition[0] * 10) / 10) + ";" + str(math.floor(targetPosition[1] * 10) / 10)
        
        return super(TravelScenarioNode, self).output(args, updateRatioCallback)
        
        
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
    