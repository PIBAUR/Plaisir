#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.scenarioNode import ScenarioNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.scenario import Scenario
from src.gui_scenario_db.src.ui import ScenarioDataBase


class ChoregraphicScenarioNode(ScenarioNode):
    nodeName = u"Sc. choré."
    
    def __init__(self, parent, canvas, position):
        super(ChoregraphicScenarioNode, self).__init__(parent, canvas, position)
        
        # ui
        self.browse_button = QPushButton(u"non défini")
        self.browse_button.clicked.connect(self.handleBrowseButtonClicked)
        self.widget.central_widget.layout().addWidget(self.browse_button)
    
    
    def handleUITimer(self, firstTime = False):
        self.browse_button.setEnabled(not self.executing)
        
        super(ChoregraphicScenarioNode, self).handleUITimer(firstTime)
        
    
    def getSpecificsData(self):
        if self.currentScenario is not None:
            return self.currentScenario.filePath
        else:
            return None
    
    
    def setSpecificsData(self, data):
        if data is not None:
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
            