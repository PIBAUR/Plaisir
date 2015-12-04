#!/usr/bin/env python
# -*- coding: utf-8 -*-

from functools import partial

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from src.scenario_lib.src.items.robot import Robot
from src.scenario_lib.src.items.nodes.scenarioNode import ScenarioNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.scenario import Scenario
from src.gui_scenario_db.src.ui import ScenarioDataBase


class ChoregraphicScenarioNode(ScenarioNode):
    nodeName = u"Sc. choré."
    
    def __init__(self, robotId, parent, canvas, position):
        super(ChoregraphicScenarioNode, self).__init__(robotId, parent, canvas, position)
        
        self.isMastering = False
        
        # ui
        self.browse_button = QPushButton(u"non défini")
        self.browse_button.clicked.connect(self.handleBrowseButtonClicked)
        self.widget.central_widget.layout().addWidget(self.browse_button)
        
        self.mastering_checkBox = QCheckBox(u"mastering")
        self.mastering_checkBox.stateChanged.connect(partial(self.handleMasteringCheckBox))
        self.widget.central_widget.layout().addWidget(self.mastering_checkBox)
    
    
    def handleUITimer(self, firstTime = False):
        self.browse_button.setEnabled(not self.executing)
        
        super(ChoregraphicScenarioNode, self).handleUITimer(firstTime)
        
    
    def handleMasteringCheckBox(self):
        self.isMastering = self.mastering_checkBox.isChecked()
        
        
    def output(self, args, updateRatioCallback):
        result = ScenarioNode.output(self, args, updateRatioCallback)
        
        if self.isMastering and self.robotId == Robot.DEFAULT_ROBOT_ID:
            for nodeInstance in self.canvas.nodesInstances:
                if nodeInstance.__class__.nodeName == "Play" and nodeInstance.robotId != self.robotId:
                    nodeInstance.playScenario(self.id)
                    
        
        return result
    
    
    def getSpecificsData(self):
        if self.currentScenario is not None:
            return self.currentScenario.filePath, self.isMastering
        else:
            return None
    
    
    def setSpecificsData(self, data):
        if data is not None:
            if type(data) == list:
                self.openScenario(data[0])
                self.mastering_checkBox.setChecked(data[1])
            else:
                self.openScenario(data)
    
    
    def handleBrowseButtonClicked(self, event):
        ScenarioDataBase(self.handleBrowseScenarioEnded)
    
    
    def handleBrowseScenarioEnded(self, filePathToOpen):
        if filePathToOpen is not None:
            self.openScenario(filePathToOpen)
        
        
    def openScenario(self, filePath):
        # set scenario
        self.currentScenario = Scenario.loadFile(filePath, False)
        self.currentScenario.filePath = filePath
        
        # change display
        self.browse_button.setText(self.currentScenario.name.decode("utf-8"))
            