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
        
        self.timer = QTimer(self.widget)
        self.timer.timeout.connect(self.handleTimer)
        self.timer.setInterval(50)
    
    
    def output(self, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        if self.currentScenario is None:
            raise NodeException(u"aucun scénario n'a été chargé")
        
        self.currentTime = float(0)
        self.startExecution(0)
        
        self.handleTimer(True)
        self.timer.start()
        
        return self.currentScenario
    
    
    def stop(self):
        super(ScenarioNode, self).stop()
        
        self.timer.stop()
        
    
    def getSpecificsData(self):
        if self.currentScenario is not None:
            return self.currentScenario.filePath
        else:
            return None
    
    
    def setSpecificsData(self, data):
        if data is not None:
            self.openScenario(data)
    
    
    def handleTimer(self, firstTime = False):
        if not firstTime:
            self.currentTime += .051
        if self.currentTime >= 1:
            self.currentTime = 1
            self.timer.stop()
        
        if self.currentTime >= 1:
            self.stopExecution()
            self.updateCallback(1, True)
        else:
            self.updateCallback(self.currentTime, False)
            self.setTimelineValue(self.currentTime)
        
    
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
            