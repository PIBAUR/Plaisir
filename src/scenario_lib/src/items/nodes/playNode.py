#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4.QtGui import *

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException

class PlayNode(DiagramNode):
    nodeName = "Play"
    nodeCategory = ""
    
    maxInputs = 1
    minInputs = 1
    hasOutput = 0
    inputGroup = [""]
    
    def __init__(self, parent, canvas, position):
        super(PlayNode, self).__init__(parent, canvas, position)
        
        self.playingScenario = None
        
        self.playButton = QPushButton("Play !")
        self.playButton.clicked.connect(self.handlePlayButtonClicked)
        self.widget.central_widget.layout().addWidget(self.playButton)
        self.playingScenarioLabel = QLabel("")
        self.widget.central_widget.layout().addWidget(self.playingScenarioLabel)
    
    
    def output(self):
        inputs = super(PlayNode, self).output()
        
        return inputs[0].output(self.updateRatio, self.updateOutput)
    
    
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass
    
    
    def updateOutput(self, output):
        self.playScenario(output)
    
    
    def updateRatio(self, ratio):
        if ratio == 1:
            self.playingScenario = None
            self.playButton.setEnabled(True)
            self.setTimelineValue(0)
            self.playingScenarioLabel.setText("")
        
        self.setTimelineValue(ratio)
    
    
    def playScenario(self, scenario):
        try:
            self.playingScenario = scenario
            self.playingScenarioLabel.setText(self.playingScenario.name)
        except NodeException, e:
            print "Erreur: " + e.message
            self.playButton.setEnabled(True)
    
    
    def handlePlayButtonClicked(self, event):
        self.playButton.setEnabled(False)
        
        self.playScenario(self.output())