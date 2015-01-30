#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.scenario import Scenario


class ScenarioNode(DiagramNode):
    nodeName = "Scenario"
    nodeCategory = ""
    
    maxInputs = 0
    minInputs = 0
    hasOutput = 1
    inputGroup = []
    
    def __init__(self, parent, canvas, position):
        super(ScenarioNode, self).__init__(parent, canvas, position)
        
        self.currentScenario = None
        
        self.browse_button = QPushButton("Parcourir ...")
        self.browse_button.clicked.connect(self.handleBrowseButtonClicked)
        self.widget.central_widget.layout().addWidget(self.browse_button)
    
    
    def output(self, updateRatioCallback, updateOutputCallback):
        self.updateCallback = updateRatioCallback
        self.updateOutputCallback = updateOutputCallback
        
        if self.currentScenario is None:
            raise NodeException(u"aucun scénario n'a été chargé")
        
        self.time = 0.
        self.timer = QTimer(self.widget)
        self.timer.timeout.connect(self.handleTimer)
        self.handleTimer()
        self.timer.setInterval(50)
        self.timer.start()
        
        return self.currentScenario
    
    
    def getSpecificsData(self):
        if self.currentScenario is not None:
            return self.currentScenario.filePath
        else:
            return None
    
    
    def setSpecificsData(self, data):
        if data is not None:
            self.openScenario(data)
    
    
    def handleTimer(self):
        self.time += .1
        if self.time >= 1:
            self.time = 1
            self.timer.stop()
            self.setTimelineValue(0)
        
        self.updateCallback(self.time)
        self.setTimelineValue(self.time)
        
    
    def handleBrowseButtonClicked(self, event):
        # hide and show because of a bug which shows a blank qfiledialog
        self.canvas.hide()
        filePathToOpen = str(QFileDialog.getOpenFileName(self.widget, u"Ouvrir un scénario", "", u"Scénario: *.sce (*.sce)"))
        self.canvas.show()
        
        if filePathToOpen != "":
            self.openScenario(filePathToOpen)
    
    
    def openScenario(self, filePath):
        # set scenario
        self.currentScenario = Scenario.loadFile(filePath, False)
        self.currentScenario.filePath = filePath
        
        # change display
        self.browse_button.hide()
        
        scenarioLabel = QLabel(self.currentScenario.name)
        scenarioLabel.setAlignment(Qt.AlignCenter)
        self.widget.central_widget.layout().addWidget(scenarioLabel)
            